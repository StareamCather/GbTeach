"""Only replace the two raise lines. Read lesson.md before looking at reference.md."""
import numpy as np


def forward(X,W1,b1,W2,b2):
    # TODO1: normalized batch [N,2] -> tanh hidden [N,12] -> linear force/8N [N].
    raise NotImplementedError('TODO1: implement the two-layer MLP forward pass')


def mse_derivative(prediction,target):
    # TODO2: derivative of (prediction-target)^2 BEFORE the batch mean.
    # network.backward performs /N exactly once.
    raise NotImplementedError('TODO2: implement the scalar squared-error derivative')
