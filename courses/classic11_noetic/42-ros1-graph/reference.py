def velocity(x0,x1,t0,t1):
    if t1 <= t0: raise ValueError("Need increasing timestamps")
    return (x1-x0)/(t1-t0)
