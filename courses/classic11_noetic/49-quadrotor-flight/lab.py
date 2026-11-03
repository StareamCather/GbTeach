#!/usr/bin/env python3
"""Closed-loop hover and bounded disturbance recovery on the native ODE body."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import uuid

import numpy as np

from controller import ARM_M, REACTION_PER_THRUST_M, ROTOR_MAX, TARGET_Z, Cascade
from plant import Plant, require
from reference import altitude_acceleration, mix


STATE_FIELDS = ['x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad', 'vx_m_s', 'vy_m_s',
                'vz_m_s', 'wx_body_rad_s', 'wy_body_rad_s', 'wz_body_rad_s']
INITIAL_ALTITUDES_M = (.45, 1.55)
ACTIONS_PER_TRIAL = 400
STEPS_PER_ACTION = 20
GUST_START = 100
GUST_END = 175
GUST_FORCE_N = (.45, -.25)
TERMINAL_WINDOW_S = 1.5


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_learner():
    path = Path('learner.py').resolve()
    spec = importlib.util.spec_from_file_location('gazeboteach_personal_a49_learner', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_student(module=None):
    module = module or load_learner()
    for values in [(9.81, 0, 0, 0), (10, .08, -.06, .01), (8, -.04, .02, -.01)]:
        np.testing.assert_allclose(module.mix(*values), mix(*values), atol=1e-9)
    for values in [(1, .3, .2), (-.3, -.4, -.1)]:
        np.testing.assert_allclose(module.altitude_acceleration(*values), altitude_acceleration(*values), atol=1e-9)
    return {'student_algebra_checks': True, 'assignment_pass': False,
            'reason': 'Complete both real flight trials using the personal controller'}


def wind_for_step(step):
    return GUST_FORCE_N if GUST_START <= step < GUST_END else (0., 0.)


def summarize_trial(rows):
    states = np.array([row['after']['state'] for row in rows])
    times = np.array([row['elapsed_sim_s'] for row in rows])
    intervals = np.array([row['after']['sim_time_s']-row['before']['sim_time_s'] for row in rows])
    tail = states[times > times[-1]-TERMINAL_WINDOW_S+1e-9]
    pre_gust = np.array(rows[GUST_START]['before']['state'])[:2]
    gust_and_recovery = states[GUST_START:, :2]-pre_gust
    return {'terminal_z_rms': float(np.sqrt(np.mean((tail[:, 2]-TARGET_Z)**2))),
            'terminal_xy_rms': float(np.sqrt(np.mean(np.sum(tail[:, :2]**2, axis=1)))),
            'terminal_attitude_rms': float(np.sqrt(np.mean(tail[:, 3:6]**2))),
            'terminal_sim_window_s': TERMINAL_WINDOW_S, 'terminal_samples': len(tail),
            'actual_sim_interval_min_s': float(intervals.min()), 'actual_sim_interval_max_s': float(intervals.max()),
            'actual_flight_duration_s': float(times[-1]),
            'peak_horizontal_excursion_from_pre_gust_m': float(np.max(np.linalg.norm(gust_and_recovery, axis=1))),
            'min_altitude_m': float(states[:, 2].min()), 'max_altitude_m': float(states[:, 2].max()),
            'gust_action_count': sum(row['wind_force_n'] != [0., 0.] for row in rows),
            'saturated_action_count': sum(bool(row['control']['saturated_rotors']) for row in rows)}


def trial(plant, initial_z, learner, path):
    record = {'initial_z': initial_z, 'requested_initial_pose': [0., 0., initial_z, .1, 0., .25],
              'complete': False, 'trace': []}
    try:
        initial = plant.reset(initial_z, roll=.1, yaw=.25)
        state = initial['native']
        record.update(actual_initial_state=state, reset_native_reply=initial['native_reply'])
        start_time = state['sim_time_s']
        # The one-step reset supplies the native physics dt even before the first action.
        dt = STEPS_PER_ACTION*initial['native_reply']['physics_dt_s']
        controller = Cascade(learner)
        for step in range(ACTIONS_PER_TRIAL):
            control = controller.compute(state['state'], dt)
            wind = wind_for_step(step)
            outcome = plant.step(control['rotors_n'], wind, STEPS_PER_ACTION)
            transition = outcome['transition']
            before, after = transition['before'], transition['after']
            record['trace'].append({'step': step, 'elapsed_sim_s': after['sim_time_s']-start_time,
                                    'controller_dt_s': dt, 'control': control,
                                    'applied_rotors_n': transition['applied_rotors_n'],
                                    'wind_force_n': transition['wind_force_n'],
                                    'before': before, 'after': after, 'transition': transition,
                                    'independent_native_readback': outcome['independent_native_readback']})
            state = outcome['native']
            dt = after['sim_time_s']-before['sim_time_s']
            altitude = state['state'][2]
            tilt = max(abs(angle) for angle in state['state'][3:5])
            require(.18 <= altitude <= 2.5 and tilt <= .65,
                    'Native flight left the altitude/tilt envelope at action '+str(step))
        record.update(summarize_trial(record['trace']))
        record['complete'] = True
        require(record['gust_action_count'] == GUST_END-GUST_START, 'Wrong number of native gust actions')
        require(record['terminal_z_rms'] < .09 and record['terminal_xy_rms'] < .14
                and record['terminal_attitude_rms'] < .09,
                'Hover/recovery thresholds exceeded: z %.5f m, xy %.5f m, attitude %.5f rad'
                % (record['terminal_z_rms'], record['terminal_xy_rms'], record['terminal_attitude_rms']))
        return {key: value for key, value in record.items() if key != 'trace'}
    finally:
        with path.open('a', encoding='utf-8') as output:
            output.write(json.dumps(record, allow_nan=False, separators=(',', ':'))+'\n')


def experiment(student=False):
    learner = load_learner() if student else None
    if learner:
        check_student(learner)
    directory = Path('results')/('run-'+uuid.uuid4().hex)
    directory.mkdir(parents=True)
    trace_path = directory/'flight-trace.jsonl'
    report_path = directory/'report.json'
    result = {'schema': 2, 'passed': False, 'assignment_pass': False,
              'execution_mode': 'student' if student else 'reference_demo',
              'algorithm': 'Position/attitude cascade PID/PD plus four-rotor force allocation',
              'run_directory': str(directory), 'report': str(report_path), 'trace_log': str(trace_path),
              'trials': [], 'state_fields': STATE_FIELDS,
              'mass_kg': 1, 'arm_m': ARM_M, 'reaction_torque_per_thrust_m': REACTION_PER_THRUST_M,
              'rotor_bound_n': [0., ROTOR_MAX], 'rotor_order': ['front', 'left', 'rear', 'right'],
              'gust': {'start_action': GUST_START, 'end_action_exclusive': GUST_END,
                       'world_xy_force_n': GUST_FORCE_N},
              'scope': 'Ideal thrust actuators, ground-truth state and simulated rigid-body hover',
              'source': {'lab_sha256': file_hash(__file__),
                         'controller_sha256': file_hash(Path(__file__).with_name('controller.py'))}}
    if student:
        result['source']['personal_learner_sha256'] = file_hash('learner.py')
    plant = None
    started = time.monotonic()
    try:
        with Plant() as plant:
            result.update(native_endpoint=plant.endpoint, build=plant.build_metadata,
                          native_initial_state=plant.begin['after'])
            for z in INITIAL_ALTITUDES_M:
                summary = trial(plant, z, learner, trace_path)
                result['trials'].append(summary)
                print(json.dumps({'initial_z': z, 'terminal_z_rms': summary['terminal_z_rms'],
                                  'terminal_xy_rms': summary['terminal_xy_rms'],
                                  'terminal_attitude_rms': summary['terminal_attitude_rms']}), flush=True)
            result['native_final_state'] = plant.state()['native']
            require(plant.samples == len(INITIAL_ALTITUDES_M)*ACTIONS_PER_TRIAL, 'Wrong number of native flight actions')
            require(plant.auxiliary_reset_physics_steps == len(INITIAL_ALTITUDES_M), 'Wrong number of reset steps')
        require(plant.close_report['confirmed'], 'Native pause restoration was not confirmed')
        result.update(passed=True, assignment_pass=bool(student))
        return result
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        if plant is not None:
            result.update(real_gazebo_transitions=plant.samples, pause_restoration=plant.close_report,
                          auxiliary_reset_physics_steps=plant.auxiliary_reset_physics_steps,
                          total_native_physics_steps=plant.samples*STEPS_PER_ACTION+plant.auxiliary_reset_physics_steps,
                          reset_observations=plant.resets,
                          watchdog={'client_lease_poisoned': plant.poisoned,
                                    'pause_restoration_confirmed': bool(plant.close_report and plant.close_report['confirmed']),
                                    'action_retry_on_unknown_outcome': False})
        if trace_path.is_file():
            result['trace_sha256'] = file_hash(trace_path)
        result['elapsed_wall_s'] = time.monotonic()-started
        write_json(report_path, result)
        write_json('results/flight-report.json', result)
        write_json('build/verification.json', result)
        print(json.dumps({key: result.get(key) for key in ('passed', 'assignment_pass', 'real_gazebo_transitions',
                         'report', 'error')}, ensure_ascii=False), flush=True)


def ready():
    with Plant() as plant:
        result = {'ready': True, 'native': plant.begin['after'], 'endpoint': plant.endpoint,
                  'source_sha256': plant.source_sha256}
    require(plant.close_report['confirmed'], 'Readiness did not restore original pause')
    result['pause_restoration'] = plant.close_report
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['demo', 'student', 'student-check', 'ready'])
    args = parser.parse_args()
    if args.command == 'student-check':
        print(json.dumps(check_student(), ensure_ascii=False))
    elif args.command == 'ready':
        print(json.dumps(ready(), ensure_ascii=False))
    else:
        experiment(args.command == 'student')
