import numpy as np

def _rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def _pose(R, p):
    return np.block([[R, p.reshape(-1, 1)], [np.zeros((1, 2)), 1]])

def _inv_pose(T):
    R = T[:2,:2]
    p = T[:2,2]
    return np.block([[R.T, (-R.T @ p).reshape(-1, 1)], [np.zeros((1, 2)), 1]])



def make_pose(x,y, theta):
    return _pose(_rot(theta), np.array([x,y]))

def make_inv_pose(x,y,theta):
    return _inv_pose(make_pose(x,y,theta))

#============== VECTOR TF CALCULATIONS HELPERS ==============

def extract_dx(DX):
    """Extract (dx, dy, dtheta) scalars from a 3x3 SE(2) transform matrix."""
    return DX[0, 2], DX[1, 2], np.arctan2(DX[1, 0], DX[0, 0])

def make_pose_batch(xs, ys, thetas):
    """Build an (N,3,3) array from N poses."""
    N = len(xs)
    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    T = np.zeros((N, 3, 3))
    T[:, 0, 0] = cos_t;  T[:, 0, 1] = -sin_t;  T[:, 0, 2] = xs
    T[:, 1, 0] = sin_t;  T[:, 1, 1] =  cos_t;  T[:, 1, 2] = ys
    T[:, 2, 2] = 1.0
    return T
