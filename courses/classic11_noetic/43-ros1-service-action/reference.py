def controller(target,position,velocity):
    error=target-position
    damping=-8*velocity
    return max(-10,min(10,18*error+damping))
