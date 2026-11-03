"""Fill two short functions; Gazebo integration, MLP backprop and logs are supplied."""
import numpy as np


def reward_to_go(rewards,gamma=.98):
    result=np.empty(len(rewards)); accumulator=0.
    for i in range(len(rewards)-1,-1,-1):
        # TODO1: combine current reward and discounted future return.
        raise NotImplementedError('TODO1: return-to-go recursion')
        result[i]=accumulator
    return result


def mean_score(raw_actions,means,sigma):
    # TODO2: d log Normal(raw_action; mean,sigma) / d mean, elementwise.
    raise NotImplementedError('TODO2: Gaussian score with fixed standard deviation')
