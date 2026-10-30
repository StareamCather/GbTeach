"""Canonical demonstration/reference; never copied over a student's learner.py."""
import numpy as np

K = np.array([-14.14213562, -15.95141643, -92.53361631, -22.61484036])
FORCE_LIMIT = 15.0


def state_error(observation, target_x=0.0):
    angle = (observation["q1"] + np.pi) % (2*np.pi) - np.pi
    return np.array([observation["x"]-target_x, observation["xdot"], angle, observation["q1dot"]])


def feedback_force(error):
    return float(-K @ np.asarray(error, dtype=float))


def clip_force(force, force_limit=FORCE_LIMIT):
    return float(np.clip(force, -force_limit, force_limit))


def control(observation, target_x=0.0):
    return clip_force(feedback_force(state_error(observation, target_x)))
