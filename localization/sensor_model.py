import numpy as np
from scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D`
# if any error re: scan_simulator_2d occurs

from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import OccupancyGrid

import sys

np.set_printoptions(threshold=sys.maxsize)


class SensorModel:

    def __init__(self, node):
        node.declare_parameter('map_topic', "default")
        node.declare_parameter('num_beams_per_particle', 1)
        node.declare_parameter('scan_theta_discretization', 1.0)
        node.declare_parameter('scan_field_of_view', 1.0)
        node.declare_parameter('lidar_scale_to_map_scale', 1.0)

        self.map_topic = node.get_parameter('map_topic').get_parameter_value().string_value
        self.num_beams_per_particle = node.get_parameter('num_beams_per_particle').get_parameter_value().integer_value
        self.scan_theta_discretization = node.get_parameter(
            'scan_theta_discretization').get_parameter_value().double_value
        self.scan_field_of_view = node.get_parameter('scan_field_of_view').get_parameter_value().double_value
        self.lidar_scale_to_map_scale = node.get_parameter(
            'lidar_scale_to_map_scale').get_parameter_value().double_value

        ####################################
        # Adjust these parameters
        self.alpha_hit   = 0.74   # dominant — rewards correct range
        self.alpha_short = 0.07   # penalizes unexpectedly close readings
        self.alpha_max   = 0.07   # handles max-range failures
        self.alpha_rand  = 0.12   # catches random noise
        self.sigma_hit   = 8.0    # in table units (pixels); tune based on map resolution

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        node.get_logger().info("%s" % self.map_topic)
        node.get_logger().info("%s" % self.num_beams_per_particle)
        node.get_logger().info("%s" % self.scan_theta_discretization)
        node.get_logger().info("%s" % self.scan_field_of_view)

        # Precompute the sensor model table
        self.sensor_model_table = np.empty((self.table_width, self.table_width))
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization)

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_subscriber = node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """

        max_range = self.table_width - 1  # e.g. 200

        d_vals = np.arange(self.table_width)  # true ranges  (0..200)
        r_vals = np.arange(self.table_width)  # measured ranges (0..200)

        # Broadcast: d is (table_width, 1), r is (1, table_width)
        d = d_vals[:, np.newaxis].astype(float)
        r = r_vals[np.newaxis, :].astype(float)

        # --- p_hit ---
        p_hit = np.exp(-0.5 * (r - d) ** 2 / (self.sigma_hit ** 2))
        p_hit /= (self.sigma_hit * np.sqrt(2 * np.pi))
        # Use 2D boolean masks
        p_hit = np.where((r >= 0) & (r <= max_range), p_hit, 0.0)
        # Normalize each row
        p_hit_sum = p_hit.sum(axis=1, keepdims=True)
        p_hit_sum[p_hit_sum == 0] = 1
        p_hit = p_hit / p_hit_sum

        # --- p_short ---
        lambda_short = 1.0
        p_short = lambda_short * np.exp(-lambda_short * r)
        # Use 2D boolean mask — r must be broadcast against d for the r <= d condition
        p_short = np.where((r >= 0) & (r <= d), p_short, 0.0)
        p_short_sum = p_short.sum(axis=1, keepdims=True)
        p_short_sum[p_short_sum == 0] = 1
        p_short = p_short / p_short_sum

        # p_max: point mass at max_range
        p_max = np.zeros((self.table_width, self.table_width))
        p_max[:, -1] = 1.0  # r == max_range

        # p_rand: uniform over [0, max_range]
        p_rand = np.ones((self.table_width, self.table_width)) / self.table_width

        # Combine
        self.sensor_model_table = (
            self.alpha_hit   * p_hit   +
            self.alpha_short * p_short +
            self.alpha_max   * p_max   +
            self.alpha_rand  * p_rand
        )

        # Normalize each row (each true range d) so probabilities sum to 1
        row_sums = self.sensor_model_table.sum(axis=1, keepdims=True)
        row_sums[row_sums == 0] = 1
        self.sensor_model_table /= row_sums


    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. THIS IS Z_K. Each range in Z_K is Z_K^i

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        # Ray-trace expected ranges for every particle: shape (N, num_beams)
        scans = self.scan_sim.scan(particles)

        # Convert to table indices by scaling to [0, table_width-1]
        # scans and observation are in meters; convert using resolution
        max_range_px = self.table_width - 1

        # Scale computed ranges and observations to pixel/table units
        scans_px = np.clip(
            scans / (self.resolution * self.lidar_scale_to_map_scale),
            0, max_range_px
        ).astype(int)

        # Downsample observation to num_beams_per_particle beams
        # (observation is the full scan; we need to subsample to match scans)
        obs_indices = np.linspace(0, len(observation) - 1, self.num_beams_per_particle, dtype=int)
        obs = observation[obs_indices]

        obs_px = np.clip(
            obs / (self.resolution * self.lidar_scale_to_map_scale),
            0, max_range_px
        ).astype(int)  # shape: (num_beams,)
        # Look up probabilities from precomputed table
        # scans_px: (N, num_beams), obs_px: (num_beams,)
        # Table is indexed [true_range_d, measured_range_r]
        probs = self.sensor_model_table[scans_px, obs_px[np.newaxis, :]]
        # probs shape: (N, num_beams)

        # Multiply probabilities across beams (log-sum for numerical stability)
        log_probs = np.log(probs + 1e-300).sum(axis=1)
        probabilities = np.exp(log_probs)

        return probabilities


    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        quat = [origin_o.x, origin_o.y, origin_o.z, origin_o.w]
        yaw = R.from_quat(quat).as_euler("xyz")[2]

        origin = (origin_p.x, origin_p.y, yaw)

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
