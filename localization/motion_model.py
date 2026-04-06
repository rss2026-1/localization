import numpy as np


class MotionModel:

    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Motion Model Initialized")

        self.node.declare_parameter("deterministic", False)
        #true if want to remove noise (actual), false if sim
        self.deterministic = self.node.get_parameter("deterministic").value

        # Bicycle model parameters
        self.node.declare_parameter("wheelbase", 0.325)        # meters, MIT RACECAR
        self.node.declare_parameter("rear_axle_fraction", 0.5) # lr/L, 0.5 = vehicle center
        self.node.declare_parameter("sigma_v", 0.1)           # m/s noise on velocity
        self.node.declare_parameter("sigma_delta", 0.1)       # rad noise on steering angle

        self.wheelbase          = self.node.get_parameter("wheelbase").value
        self.rear_axle_fraction = self.node.get_parameter("rear_axle_fraction").value
        self.sigma_v            = self.node.get_parameter("sigma_v").value
        self.sigma_delta        = self.node.get_parameter("sigma_delta").value


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

    # def evaluate1(self, particles, odometry):
    #     """
    #     Update the particles to reflect probable
    #     future states given the odometry data.

    #     args:
    #         particles: An Nx3 matrix of the form:

    #             [x0 y0 theta0]
    #             [x1 y0 theta1]
    #             [    ...     ]
    #         odometry: [dx, dy, dtheta] motion delta

    #     returns:
    #         particles: An updated matrix of the same size
    #     """
    #     dx, dy, dtheta = odometry
    #     N = len(particles)

    #     if self.deterministic:
    #         dx_n     = np.full(N, dx)
    #         dy_n     = np.full(N, dy)
    #         dtheta_n = np.full(N, dtheta)
    #     else:
    #         dx_n     = dx     + np.random.normal(0.0, 0.05, N)
    #         dy_n     = dy     + np.random.normal(0.0, 0.05, N)
    #         dtheta_n = dtheta + np.random.normal(0.0, 0.02, N)

    #     DX_batch = self.make_pose_batch(dx_n, dy_n, dtheta_n)

    #     T_particles = self.make_pose_batch(particles[:,0], particles[:,1], particles[:,2])
    #     T_new = T_particles @ DX_batch

    #     particles[:,0] = T_new[:,0,2]
    #     particles[:,1] = T_new[:,1,2]
    #     particles[:,2] = np.arctan2(T_new[:,1,0], T_new[:,0,0])

    #     return particles

    def evaluate(self, particles, odometry, v=None, steering_angle=None, dt=None):
        """
        Update particles using the kinematic bicycle model when Ackermann data
        is available, otherwise falls back to nominal evaluate function.
        """
        N = len(particles)

        use_bicycle = (v is not None) and (steering_angle is not None) and (dt is not None)

        if self.deterministic:
            L  = self.wheelbase
            lr = L * self.rear_axle_fraction

            if self.deterministic:
                v_n     = np.full(N, v)
                delta_n = np.full(N, steering_angle)
            else:
                v_n     = v              + np.random.normal(0.0, self.sigma_v,     N)
                delta_n = steering_angle + np.random.normal(0.0, self.sigma_delta, N)

            # Slip angle at the vehicle reference point
            beta = np.arctan(np.tan(delta_n) * lr / L)

            dx_n     = v_n * np.cos(beta) * dt
            dy_n     = v_n * np.sin(beta) * dt
            dtheta_n = (v_n / L) * np.cos(beta) * np.tan(delta_n) * dt

        else:
            dx, dy, dtheta = odometry

            if self.deterministic:
                dx_n     = np.full(N, dx)
                dy_n     = np.full(N, dy)
                dtheta_n = np.full(N, dtheta)
            else:
                dx_n     = dx     + np.random.normal(0.0, self.sigma_v,     N)
                dy_n     = dy     + np.random.normal(0.0, self.sigma_v,     N)
                dtheta_n = dtheta + np.random.normal(0.0, self.sigma_delta, N)

        DX_batch    = self.make_pose_batch(dx_n, dy_n, dtheta_n)
        T_particles = self.make_pose_batch(particles[:,0], particles[:,1], particles[:,2])
        T_new = T_particles @ DX_batch

        particles[:,0] = T_new[:,0,2]
        particles[:,1] = T_new[:,1,2]
        particles[:,2] = np.arctan2(T_new[:,1,0], T_new[:,0,0])

        return particles
