def transform_point(R,t,p):
    return [sum(R[i][j]*p[j] for j in range(3))+t[i] for i in range(3)]

def stamp_delta(t0,t1):
    return abs(t0-t1)
