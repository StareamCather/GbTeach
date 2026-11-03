"""Check the REINFORCE algebra independently of Gazebo runtime acceptance."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np


COURSE = Path(__file__).resolve().parents[1]/'courses/classic11_noetic/48-mlp-reinforcement'


def load(name):
    spec = importlib.util.spec_from_file_location('gazeboteach_a48_'+name, str(COURSE/(name+'.py')))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


network, plant, reference = (load(name) for name in ('network', 'plant', 'reference'))
with mock.patch.dict(sys.modules, {'network': network, 'plant': plant, 'reference': reference}):
    lab = load('lab')


class ReinforceTests(unittest.TestCase):
    def test_return_to_go_matches_explicit_discounted_suffixes(self):
        rewards = [-.3, -.2, -.1, -.6]
        gamma = .81
        expected = [sum(gamma**(j-i)*rewards[j] for j in range(i, len(rewards)))
                    for i in range(len(rewards))]
        np.testing.assert_allclose(reference.reward_to_go(rewards, gamma), expected)

    def test_gradient_ascent_matches_weighted_log_density_finite_differences(self):
        inputs = np.array([[-.4, .2], [-.3, .3], [.5, -.1], [.4, -.2]])
        model = network.MLP(seed=19)
        means = model.forward(inputs)
        actions = np.array([.3, .9, -.2, -1.2])
        rewards = [-.5, -.4, -.1, -.3]
        baseline = np.array([-.1, -.05])
        records = [{'reward': reward, 'raw_action_n': action, 'mean_n': mean}
                   for reward, action, mean in zip(rewards, actions, means)]
        derivative, _ = lab.batch_derivative([records[:2], records[2:]], baseline)
        gamma = lab.GAMMA
        # Fixed observed returns, raw samples and baseline. Perturb only the candidate means.
        coefficient = np.array([rewards[0]+gamma*rewards[1]-baseline[0],
                                gamma*(rewards[1]-baseline[1]),
                                rewards[2]+gamma*rewards[3]-baseline[0],
                                gamma*(rewards[3]-baseline[1])])
        def objective():
            return np.mean(coefficient*(-.5*((actions-model.forward(inputs))/lab.SIGMA)**2))

        parameters = {key: getattr(model, key).copy() for key in ('W1', 'b1', 'W2', 'b2')}
        expected = {}
        epsilon = 1e-6
        for key, original in parameters.items():
            current = getattr(model, key)
            numeric = np.zeros_like(original)
            for index in np.ndindex(original.shape):
                current[index] = original[index]+epsilon
                plus = objective()
                current[index] = original[index]-epsilon
                minus = objective()
                current[index] = original[index]
                numeric[index] = (plus-minus)/(2*epsilon)
            expected[key] = numeric
        rate = .035
        self.assertLess(model.backward(inputs, derivative, rate, ascent=True), 5)
        for key, original in parameters.items():
            np.testing.assert_allclose((getattr(model, key)-original)/rate, expected[key], atol=2e-9)

    def test_algebra_only_does_not_complete_assignment(self):
        self.assertFalse(lab.check_student(reference)['assignment_pass'])
        with self.assertRaises(NotImplementedError):
            lab.check_student(load('learner'))


if __name__ == '__main__':
    unittest.main()
