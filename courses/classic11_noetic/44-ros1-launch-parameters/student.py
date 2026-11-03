import math

def validate_force(value):
    # TODO1: int/float are accepted; bool is not a physical numeric setting.
    valid_type = None
    # TODO2: short-circuit the type check, then finite and .2 <= value <= 3.
    in_range = None
    if valid_type is None or in_range is None:raise NotImplementedError('Fill the two validation expressions')
    if not valid_type or not in_range:raise ValueError('Force must be a finite number in [.2,3] N')
    return float(value)
