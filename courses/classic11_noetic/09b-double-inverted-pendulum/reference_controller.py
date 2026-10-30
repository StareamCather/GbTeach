"""Canonical double-pole reference; never automatically replaces learner.py."""
import numpy as np

K = np.array([7.07106781, 11.24116542, -161.66415569,
              3.24033851, 233.38706339, 31.37395393])
FORCE_LIMIT = 20.0


def state_error(observation, target_x=0.0):
    angle1 = (observation["q1"]+np.pi) % (2*np.pi)-np.pi
    angle2 = (observation["q1"]+observation["q2"]+np.pi) % (2*np.pi)-np.pi
    return np.array([observation["x"]-target_x, observation["xdot"], angle1,
                     observation["q1dot"], angle2, observation["q1dot"]+observation["q2dot"]])


def feedback_force(error):
    return float(-K @ np.asarray(error, dtype=float))


def clip_force(force, force_limit=FORCE_LIMIT):
    return float(np.clip(force, -force_limit, force_limit))


def control(observation, target_x=0.0):
    return clip_force(feedback_force(state_error(observation, target_x)))
