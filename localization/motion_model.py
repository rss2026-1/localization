import numpy as np


class MotionModel:

    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Motion Model Initialized")

        self.node.declare_parameter("is_sim", False)
        self.is_sim = self.node.get_parameter("is_sim").value


    def make_pose_batch(self, xs, ys, thetas):
        """Build an (N,3,3) array from N poses."""
        N = len(xs)
        cos_t = np.cos(thetas)
        sin_t = np.sin(thetas)
        T = np.zeros((N, 3, 3))
        T[:, 0, 0] = cos_t;  T[:, 0, 1] = -sin_t;  T[:, 0, 2] = xs
        T[:, 1, 0] = sin_t;  T[:, 1, 1] =  cos_t;  T[:, 1, 2] = ys
        T[:, 2, 2] = 1.0
        return T

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]
            odometry: [dx, dy, dtheta] motion delta

        returns:
            particles: An updated matrix of the same size
        """
        dx, dy, dtheta = odometry
        N = len(particles)

        if not self.is_sim:
            dx_n     = np.full(N, dx)
            dy_n     = np.full(N, dy)
            dtheta_n = np.full(N, dtheta)
        else:
            dx_n     = dx     + np.random.normal(0.0, 0.05, N)
            dy_n     = dy     + np.random.normal(0.0, 0.05, N)
            dtheta_n = dtheta + np.random.normal(0.0, 0.02, N)

        DX_batch = self.make_pose_batch(dx_n, dy_n, dtheta_n)

        T_particles = self.make_pose_batch(particles[:,0], particles[:,1], particles[:,2])
        T_new = T_particles @ DX_batch

        particles[:,0] = T_new[:,0,2]
        particles[:,1] = T_new[:,1,2]
        particles[:,2] = np.arctan2(T_new[:,1,0], T_new[:,0,0])

        return particles
