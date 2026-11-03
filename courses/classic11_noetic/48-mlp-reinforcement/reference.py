"""Complete reference algebra for the two personal exercise blanks."""
import numpy as np


def reward_to_go(rewards, gamma=.98):
    result = np.empty(len(rewards))
    accumulator = 0.
    for index in range(len(rewards)-1, -1, -1):
        accumulator = rewards[index]+gamma*accumulator
        result[index] = accumulator
    return result


def mean_score(raw_actions, means, sigma):
    return (np.asarray(raw_actions)-np.asarray(means))/(sigma*sigma)
