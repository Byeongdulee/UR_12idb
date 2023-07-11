from numpy import (dot, arccos, clip)
from numpy.linalg import norm

def ind2sub(ind, array_shape):
    rows = int(ind / array_shape[1])
    cols = (int(ind) % array_shape[1]) # or numpy.mod(ind.astype('int'), array_shape[1])
    return (rows, cols)

def sub2ind(rows, cols, array_shape):
    return rows*array_shape[1] + cols

def get_angle_vectors(u, v):
    c = dot(u,v)/norm(u)/norm(v) # -> cosine of the angle
    angle = arccos(clip(c, -1, 1)) # if you really want the angle
    return angle
