import math

def validate_force(value):
    valid_type = isinstance(value,(int,float)) and not isinstance(value,bool)
    in_range = valid_type and math.isfinite(value) and .2 <= value <= 3
    if not valid_type or not in_range:raise ValueError('Force must be a finite number in [.2,3] N')
    return float(value)
