"""A47 only: mathematical gradient, provenance and real-client failure boundaries."""
import importlib.util
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np


SOURCE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/47-mlp-imitation'


def load(name):
    spec = importlib.util.spec_from_file_location('gazeboteach_a47_'+name, str(SOURCE/(name+'.py')))
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


network = load('network'); plant = load('plant')
with mock.patch.dict(sys.modules, {'network': network, 'plant': plant}): lab = load('lab')


def snapshot(iterations=100, sim=.1, x=.2, v=-.1, callbacks=50):
    return dict(iterations=iterations, sim_time_s=sim, x_m=x, v_m_s=v,
                paused=True, callbacks=callbacks)


def transition():
    return {'before': snapshot(), 'after': snapshot(120, .12, .19, -.09, 70),
            'physics_dt_s': .001, 'steps': 20, 'force_n': .5}


class MLPImitationBoundaries(unittest.TestCase):
    def test_all_49_backprop_parameters_match_independent_finite_differences(self):
        model = network.MLP(); X = np.array([[.2, -.3], [-.7, .4], [.1, .8]])
        y = np.array([.1, -.2, .3]); epsilon = 1e-6; rate = .003
        original = {name: getattr(model, name).copy() for name in ('W1', 'b1', 'W2', 'b2')}
        numeric = {}
        for name, values in original.items():
            gradient = np.zeros_like(values)
            for index in np.ndindex(values.shape):
                getattr(model, name)[index] += epsilon
                plus = np.mean((model.forward(X)-y)**2)
                getattr(model, name)[index] -= 2*epsilon
                minus = np.mean((model.forward(X)-y)**2)
                getattr(model, name)[index] = values[index]
                gradient[index] = (plus-minus)/(2*epsilon)
            numeric[name] = gradient
        norm = model.backward(X, 2*(model.forward(X)-y), rate)
        self.assertLess(norm, 5)
        for name, values in original.items():
            actual = (values-getattr(model, name))/rate
            np.testing.assert_allclose(actual, numeric[name], atol=2e-9, rtol=1e-6)

    def test_global_gradient_clip_limits_the_whole_parameter_update(self):
        model = network.MLP(); original = {name: getattr(model, name).copy() for name in ('W1', 'b1', 'W2', 'b2')}
        norm = model.backward(np.array([[.5, .2], [-.4, .9]]), np.array([1e5, -1e5]), .01)
        self.assertGreater(norm, 5)
        update_norm = np.sqrt(sum(np.sum((original[name]-getattr(model, name))**2) for name in original))
        self.assertAlmostEqual(update_norm, .01*5, places=10)

    def test_fixed_scales_are_shift_equivariant_and_saved_without_pickle(self):
        a = plant.features({'x': -.2, 'v': .4}, .3)
        b = plant.features({'x': .5, 'v': .4}, 1.)
        np.testing.assert_allclose(a, [.5, .2]); np.testing.assert_allclose(a, b)
        self.assertEqual(plant.NORMALIZATION['method'], 'fixed_physical_scales_not_fitted')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'policy.npz'; model = network.MLP()
            model.save(path, normalization=plant.NORMALIZATION)
            with np.load(path, allow_pickle=False) as data:
                self.assertEqual(json.loads(str(data['normalization_json'])), plant.NORMALIZATION)
            np.testing.assert_array_equal(network.MLP().load(path).forward(a), model.forward(a))

    def test_split_keeps_entire_episodes_separate_and_rejects_unknown_episode(self):
        rows = [{'episode': episode} for episode in range(10) for unused in range(60)]
        train, held = lab.split_episodes(rows)
        self.assertEqual(int(train.sum()), 480); self.assertEqual(int(held.sum()), 120)
        self.assertEqual(set(row['episode'] for row, keep in zip(rows, held) if keep), {8, 9})
        with self.assertRaises(RuntimeError): lab.split_episodes(rows+[{'episode': 11}])
        self.assertEqual(lab.UNSEEN_TARGETS, [-.3, .3])

    def test_local_action_validation_does_not_round_clip_or_transmit_bad_values(self):
        instance = object.__new__(plant.Plant); instance._request = mock.Mock()
        for force, steps in [(float('nan'), 20), (float('inf'), 20), (8.1, 20), (True, 20),
                             (.5, 20.), (.5, 2.5), (.5, True), (.5, 0), (.5, 201)]:
            with self.assertRaises(RuntimeError): instance.step(force, steps)
        instance._request.assert_not_called()

    def test_transition_rejects_unrecorded_iterations_time_force_or_callbacks(self):
        plant.validate_transition(transition(), .5, 20)
        for field, value in [('iterations', 121), ('sim_time_s', .121), ('callbacks', 71), ('paused', False)]:
            reply = transition(); reply['after'][field] = value
            with self.assertRaises(RuntimeError): plant.validate_transition(reply, .5, 20)
        with self.assertRaises(RuntimeError): plant.validate_transition(transition(), .6, 20)

    def test_independent_readback_detects_between_request_coast(self):
        instance = object.__new__(plant.Plant); instance.last = snapshot(); instance.poisoned = False; instance.samples = 0
        readback = snapshot(121, .121, .19, -.09, 71)
        instance._request = mock.Mock(side_effect=[transition(), {'after': readback}])
        with self.assertRaisesRegex(RuntimeError, 'unrecorded coast'): instance.step(.5)
        self.assertTrue(instance.poisoned); self.assertEqual(instance.samples, 0)

    def test_wall_timeout_is_absolute_even_when_peer_trickles_and_never_replays_end(self):
        local, remote = socket.socketpair(); local.setblocking(False)
        instance = object.__new__(plant.Plant)
        instance.connection = local; instance.timeout = .06; instance.next_id = 1
        instance.buffer = bytearray(); instance.poisoned = False; instance.closed = False
        instance.requests = []; instance.close_report = None
        stop = threading.Event(); received = []
        def peer():
            received.append(remote.recv(1024))
            while not stop.wait(.008):
                try: remote.sendall(b' ')
                except OSError: break
        thread = threading.Thread(target=peer, daemon=True); thread.start(); started = time.monotonic()
        try:
            with self.assertRaises(plant.UnknownOutcome): instance._request('step', .5, 20)
            elapsed = time.monotonic()-started
            self.assertGreater(elapsed, .04); self.assertLess(elapsed, .3)
            self.assertTrue(instance.poisoned)
            instance.close(); self.assertFalse(instance.close_report['confirmed'])
            self.assertEqual(received, [b'1 step 0.5 20\n'])
        finally:
            stop.set(); local.close(); remote.close(); thread.join(.3)

    def test_known_native_rejection_is_not_mislabelled_unknown_mutation(self):
        local, remote = socket.socketpair(); local.setblocking(False)
        instance = object.__new__(plant.Plant)
        instance.connection = local; instance.timeout = .2; instance.next_id = 1
        instance.buffer = bytearray(); instance.poisoned = False; instance.closed = False; instance.requests = []
        reply = {'protocol': 2, 'id': 1, 'kind': 'step', 'ok': False,
                 'mutation_started': False, 'error': 'invalid request'}
        remote.sendall((json.dumps(reply)+'\n').encode('ascii'))
        try:
            with self.assertRaises(RuntimeError) as raised: instance._request('step', .5, 20)
            self.assertNotIsInstance(raised.exception, plant.UnknownOutcome)
        finally: local.close(); remote.close()

    def test_reference_algebra_is_separate_and_personal_exercise_is_unfilled(self):
        reference = load('reference')
        self.assertTrue(lab.check_student(reference)['student_algebra_checks'])
        self.assertFalse(lab.check_student(reference)['assignment_pass'])
        learner = load('learner')
        with self.assertRaises(NotImplementedError): learner.forward([[0., 0.]], None, None, None, None)


if __name__ == '__main__': unittest.main()
