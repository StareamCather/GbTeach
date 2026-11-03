"""Factor-graph practice: one scalar pose with two independent constraints."""


def whitened_residual(state, observation, sigma):
    """Return the signed residual in units of one standard deviation."""
    # TODO: subtract the observation, then scale by standard deviation.
    raise NotImplementedError("Complete whitened_residual")


def weighted_position(observations, sigmas):
    """Minimize sum_i ((x-z_i)/sigma_i)^2 for independent Gaussian factors."""
    # TODO: solve the zero-gradient equation; precision is 1/sigma**2.
    raise NotImplementedError("Complete weighted_position")
