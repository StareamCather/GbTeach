"""Coordinate and actuator invariants for the A49 controller."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np


COURSE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/49-quadrotor-flight'


def load(name):
    spec = importlib.util.spec_from_file_location('gazeboteach_a49_'+name, str(COURSE/(name+'.py')))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reference = load('reference')
with mock.patch.dict(sys.modules, {'reference': reference}):
    controller = load('controller')


class QuadrotorControllerTests(unittest.TestCase):
    def test_mix_reconstructs_force_and_torques_from_physical_lever_arms(self):
        positions = np.array([[.22, 0, 0], [0, .22, 0], [-.22, 0, 0], [0, -.22, 0]])
        yaw_per_thrust = np.array([.018, -.018, .018, -.018])
        for total, roll, pitch, yaw in [(9.81, 0, 0, 0), (10, .08, -.06, .01), (8, -.04, .02, -.01)]:
            rotors = reference.mix(total, roll, pitch, yaw)
            forces = np.column_stack((np.zeros(4), np.zeros(4), rotors))
            moments = np.sum(np.cross(positions, forces), axis=0)
            moments[2] += yaw_per_thrust@rotors
            np.testing.assert_allclose([np.sum(rotors), *moments], [total, roll, pitch, yaw], atol=1e-12)

    def test_hover_and_rotation_from_world_to_yaw_aligned_axes(self):
        state = np.zeros(12)
        state[2] = 1.
        hover = controller.Cascade().compute(state, .02)
        np.testing.assert_allclose(hover['rotors_n'], [9.81/4]*4)
        state[0] = 1.
        state[5] = np.pi/2
        tilted = controller.Cascade().compute(state, .02)
        # World -X is body +Y when yaw is +90 degrees, requiring a negative roll.
        self.assertLess(tilted['desired_roll_rad'], 0)
        self.assertAlmostEqual(tilted['desired_pitch_rad'], 0)
        self.assertLess(tilted['requested_wrench_n_nm'][3], 0)

    def test_bounds_are_visible_after_mixer_saturation(self):
        state = np.zeros(12)
        state[2] = 1.
        state[11] = 20.
        action = controller.Cascade().compute(state, .02)
        self.assertEqual(action['saturated_rotors'], [0, 1, 2, 3])
        self.assertEqual(action['rotors_n'], [0., 6., 0., 6.])
        self.assertNotAlmostEqual(action['requested_wrench_n_nm'][3], action['allocated_wrench_n_nm'][3])

    def test_integral_respects_sim_interval_and_clamp(self):
        state = np.zeros(12)
        state[2] = .5
        flight = controller.Cascade()
        self.assertAlmostEqual(flight.compute(state, .02)['altitude_integral_m_s'], .01)
        self.assertAlmostEqual(flight.compute(state, .04)['altitude_integral_m_s'], .03)
        for _ in range(100):
            bounded = flight.compute(state, .02)
        self.assertEqual(bounded['altitude_integral_m_s'], .6)


if __name__ == '__main__':
    unittest.main()
