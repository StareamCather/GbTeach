"""Point-LIO practice: equal-time grouping and a rate from actual stamps."""


def group_sizes(sorted_stamps_ns):
    """Return sizes of consecutive groups of equal acquisition timestamps."""
    # TODO: start a group when the integer timestamp changes.
    raise NotImplementedError("Complete group_sizes")


def event_rate_hz(distinct_stamps_ns):
    """Return interval count divided by elapsed seconds; 0 for fewer than two."""
    # TODO: count the intervals and subtract timestamps before converting ns.
    raise NotImplementedError("Complete event_rate_hz")
