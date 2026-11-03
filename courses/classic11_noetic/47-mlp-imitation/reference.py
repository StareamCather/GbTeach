"""Separate algebra answer; viewing/running it does not complete the assignment."""
import numpy as np


def forward(X, W1, b1, W2, b2):
    return (np.tanh(np.atleast_2d(X)@W1+b1)@W2+b2).reshape(-1)


def mse_derivative(prediction, target):
    return 2*(np.asarray(prediction)-np.asarray(target))
