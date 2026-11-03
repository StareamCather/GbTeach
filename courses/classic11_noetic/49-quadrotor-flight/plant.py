"""Bounded client: only Gazebo integrates the real, paused physical plant."""
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import socket
import stat
import struct
import time

import numpy as np


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


class UnknownOutcome(RuntimeError):
    """The one request may have reached Gazebo. Never replay it automatically."""


def finite_number(value, name):
    require(not isinstance(value, (bool, np.bool_)), name+' must not be bool')
    require(isinstance(value, (int, float, np.integer, np.floating)), name+' must be numeric')
    result = float(value)
    require(math.isfinite(result), name+' must be finite')
    return result


def validate_action(rotors, wind, steps):
    require(len(rotors) == 4 and len(wind) == 2, 'Expected four rotors and two wind components')
    rotors = tuple(finite_number(v, 'rotor force') for v in rotors)
    wind = tuple(finite_number(v, 'wind force') for v in wind)
    require(all(0 <= v <= 6 for v in rotors), 'Rotor force outside [0,6] N')
    require(all(abs(v) <= 1 for v in wind), 'Wind component outside [-1,1] N')
    require(isinstance(steps, (int, np.integer)) and not isinstance(steps, (bool, np.bool_))
            and steps == 20, 'Exactly 20 integer physics steps required')
    return rotors, wind, int(steps)


def validate_snapshot(row):
    require(isinstance(row, dict), 'Native snapshot is not an object')
    for key in ('iterations', 'callbacks'):
        require(isinstance(row.get(key), int) and not isinstance(row.get(key), bool) and row[key] >= 0,
                'Native '+key+' is invalid')
    finite_number(row.get('sim_time_s'), 'sim_time_s')
    state = row.get('state')
    require(isinstance(state, list) and len(state) == 12, 'Native state vector must have 12 values')
    for value in state: finite_number(value, 'state component')
    require(isinstance(row.get('paused'), bool), 'Native pause flag is invalid')
    return row


def same_snapshot(a, b):
    return (a['iterations'] == b['iterations'] and a['callbacks'] == b['callbacks']
            and a['paused'] == b['paused']
            and abs(a['sim_time_s']-b['sim_time_s']) <= 1e-10
            and all(abs(x-y) <= 1e-10 for x,y in zip(a['state'],b['state'])))


def validate_transition(reply, rotors, wind, steps):
    before = validate_snapshot(reply['before']); after = validate_snapshot(reply['after'])
    require(before['paused'] and after['paused'], 'Discrete transition lost the paused invariant')
    dt = finite_number(reply['physics_dt_s'], 'physics_dt_s')
    require(dt > 0, 'Non-positive native physics dt')
    require(after['iterations']-before['iterations'] == steps, 'Native iteration delta differs from requested steps')
    require(after['callbacks']-before['callbacks'] == steps, 'Native force callback count differs from requested steps')
    require(abs(after['sim_time_s']-before['sim_time_s']-steps*dt) <= max(1e-9, steps*dt*1e-8),
            'Native simulation-time delta differs from exact steps')
    require(reply['steps'] == steps, 'Native steps acknowledgement differs')
    for key, expected in (('applied_rotors_n', rotors), ('wind_force_n', wind)):
        actual = reply[key]
        require(len(actual) == len(expected) and all(abs(finite_number(a,key)-b) < 1e-12 for a,b in zip(actual,expected)),
                'Native action acknowledgement differs: '+key)
    return after


class Plant:
    def __init__(self, timeout=4.0):
        token = os.environ.get('GAZEBOTEACH_SESSION_TOKEN', '')
        require(re.fullmatch('[0-9a-fA-F]{32}', token) is not None,
                'Start this lesson and use its experiment terminal first')
        self.path = '/tmp/gt-quad-'+token+'.sock'
        details = os.lstat(self.path)
        require(stat.S_ISSOCK(details.st_mode) and details.st_uid == os.getuid()
                and stat.S_IMODE(details.st_mode) == 0o600, 'Untrusted plant socket owner/type/mode')
        self.endpoint = {'path': self.path, 'uid': details.st_uid, 'mode': oct(stat.S_IMODE(details.st_mode)),
                         'device': details.st_dev, 'inode': details.st_ino}
        self.connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.connection.settimeout(timeout)
        self.timeout = finite_number(timeout, 'timeout'); require(0 < self.timeout <= 30, 'Invalid timeout')
        self.poisoned = False; self.closed = False; self.samples = 0; self.next_id = 1
        self.reset_samples = 0; self.auxiliary_reset_physics_steps = 0
        self.buffer = bytearray(); self.last = None; self.close_report = None; self.requests = []; self.resets = []
        try:
            self.connection.connect(self.path)
            peer_pid, peer_uid, peer_gid = struct.unpack('3i', self.connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            require(peer_uid == os.getuid(), 'Native server belongs to a different UID')
            self.endpoint['peer'] = {'pid': peer_pid, 'uid': peer_uid, 'gid': peer_gid}
            self.connection.setblocking(False)
            self.begin = self._request('begin')
            self.last = validate_snapshot(self.begin['after'])
            require(self.last['paused'], 'Native begin did not confirm pause')
            require(isinstance(self.begin['original_paused'], bool), 'Original pause state missing')
            self.before_paused = self.begin['original_paused']
            metadata = json.loads(Path('build/plant-build.json').read_text(encoding='utf-8'))
            require(self.begin['source_sha256'] == metadata['source_sha256'], 'Loaded plant source differs from prepared source')
            self.source_sha256 = self.begin['source_sha256']
            self.build_metadata = metadata
        except BaseException:
            self.connection.close(); self.closed = True
            raise

    def _request(self, verb, *args):
        require(not self.poisoned and not self.closed, 'Plant connection is closed or has an unknown outcome')
        request_id = self.next_id; self.next_id += 1
        fields = [str(request_id), verb] + [format(value, '.17g') for value in (args or (0, 0))]
        payload = (' '.join(fields)+'\n').encode('ascii')
        deadline = time.monotonic()+self.timeout; sent = 0; known_rejection = False
        try:
            while sent < len(payload):
                remaining = deadline-time.monotonic()
                if remaining <= 0: raise TimeoutError('wall deadline while sending '+verb)
                if not select.select([], [self.connection], [], remaining)[1]:
                    raise TimeoutError('wall deadline while sending '+verb)
                try: count = self.connection.send(payload[sent:])
                except BlockingIOError: continue
                if not count: raise ConnectionError('Native socket closed while sending')
                sent += count
            while b'\n' not in self.buffer:
                remaining = deadline-time.monotonic()
                if remaining <= 0: raise TimeoutError('wall deadline while receiving '+verb)
                if not select.select([self.connection], [], [], remaining)[0]:
                    raise TimeoutError('wall deadline while receiving '+verb)
                try: chunk = self.connection.recv(16384)
                except BlockingIOError: continue
                if not chunk: raise ConnectionError('Native socket closed before acknowledgement')
                self.buffer.extend(chunk)
                require(len(self.buffer) <= 65536, 'Oversized native response')
            line, _, remainder = self.buffer.partition(b'\n'); self.buffer = bytearray(remainder)
            reply = json.loads(line.decode('ascii'))
            require(reply.get('protocol') == 2 and reply.get('id') == request_id and reply.get('kind') == verb,
                    'Native protocol/request identity mismatch')
            self.requests.append({'id': request_id, 'kind': verb, 'reply': reply})
            if not reply.get('ok'):
                if reply.get('mutation_started'):
                    raise UnknownOutcome('Native mutation failed after starting: '+str(reply.get('error')))
                known_rejection = True
                raise RuntimeError('Native rejected request: '+str(reply.get('error')))
            return reply
        except BaseException as error:
            # Even a partial send is uncertain. Close/EOF is the only cleanup channel;
            # sending end here could race a still-running native Step(N).
            self.poisoned = True
            if isinstance(error, (KeyboardInterrupt, SystemExit)): raise
            if sent and not known_rejection:
                raise UnknownOutcome('Request '+verb+' outcome is unknown; not replayed: '+str(error)) from error
            raise

    def _row(self, snapshot, **extra):
        return dict(state=snapshot['state'], time=snapshot['sim_time_s'],
                    native=snapshot, **extra)

    def state(self):
        reply = self._request('state'); snapshot = validate_snapshot(reply['after'])
        require(snapshot['paused'], 'Native state is not paused')
        self.last = snapshot
        return self._row(snapshot, native_reply=reply)

    def reset(self, z, roll=.1, yaw=.25):
        z, roll, yaw = (finite_number(v, key) for key,v in (('z', z), ('roll', roll), ('yaw', yaw)))
        require(.3 <= z <= 2 and abs(roll) <= .3 and abs(yaw) <= .8, 'Reset outside lesson envelope')
        reply = self._request('reset', z, roll, yaw)
        try:
            after = validate_transition(reply, (0,)*4, (0,)*2, 1)
            require(same_snapshot(self.last, reply['before']), 'Physics changed before reset')
            actual = after['state']
            requested = [0,0,z,roll,0,yaw,0,0,0,0,0,0]
            require(all(abs(a-b)<.02 for a,b in zip(actual, requested)), 'Reset readback outside one-step tolerance')
            self.reset_samples += 1; self.auxiliary_reset_physics_steps += 1
            self.resets.append({'requested_state': requested, 'actual_state': actual, 'native_reply': reply})
            self.last = after
            return self._row(after, native_reply=reply)
        except BaseException:
            self.poisoned = True
            raise

    def step(self, rotors, wind=(0,0), steps=20):
        rotors, wind, steps = validate_action(rotors, wind, steps)
        reply = self._request('step', *rotors, *wind, steps)
        try:
            after = validate_transition(reply, rotors, wind, steps)
            require(same_snapshot(self.last, reply['before']), 'Physics coasted or changed between requests')
            readback = self._request('state')
            require(same_snapshot(after, validate_snapshot(readback['after'])), 'Readback found unrecorded coast')
            self.last = after; self.samples += 1
            return self._row(after, transition=reply, independent_native_readback=readback)
        except BaseException:
            self.poisoned = True
            raise

    def close(self):
        if self.closed: return self.close_report
        try:
            if not self.poisoned:
                reply = self._request('end')
                require(reply['restored_paused'] == self.before_paused, 'Original pause state was not restored')
                self.close_report = {'confirmed': True, 'original_paused': self.before_paused, 'native_reply': reply}
            else:
                self.close_report = {'confirmed': False, 'reason': 'Unknown request: no end/retry; native EOF cleanup only'}
        finally:
            self.connection.close(); self.closed = True
        return self.close_report

    def __enter__(self):
        self.old_signals = {}
        for number in (signal.SIGTERM, getattr(signal, 'SIGHUP', signal.SIGTERM)):
            self.old_signals[number] = signal.getsignal(number)
        def cancel(number, frame): raise KeyboardInterrupt('Training cancelled by signal '+str(number))
        for number in self.old_signals: signal.signal(number, cancel)
        return self

    def __exit__(self, *args):
        try: self.close()
        finally:
            for number, handler in self.old_signals.items(): signal.signal(number, handler)

