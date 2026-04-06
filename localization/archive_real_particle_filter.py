# from localization.sensor_model import SensorModel
# from localization.motion_model import MotionModel

# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped

# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
# from sensor_msgs.msg import LaserScan

# from rclpy.node import Node
# import rclpy
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# import tf2_ros
# from tf2_ros import TransformBroadcaster

# assert rclpy


# class ParticleFilter(Node):

#     def __init__(self):
#         super().__init__("particle_filter")

#         self.declare_parameter('particle_filter_frame', "default")
#         self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

#         #  *Important Note #1:* It is critical for your particle
#         #     filter to obtain the following topic names from the
#         #     parameters for the autograder to work correctly. Note
#         #     that while the Odometry message contains both a pose and
#         #     a twist component, you will only be provided with the
#         #     twist component, so you should rely only on that
#         #     information, and *not* use the pose component.

#         # Initialize the transform broadcaster
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Initialize transform listener
#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.declare_parameter('odom_topic', "/odom")
#         self.declare_parameter('scan_topic', "/scan")

#         scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

#         self.num_particles = 200
#         self.laser_callback_freq = 20 #in Hz
#         self.laser_callback_prev_time = 0.0

#         self.particles = None

#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.laser_sub = self.create_subscription(LaserScan, scan_topic,
#                                                   self.laser_callback,
#                                                   1)

#         self.odom_sub = self.create_subscription(Odometry, odom_topic,
#                                                  self.odom_callback,
#                                                  1)

#         #  *Important Note #2:* You must respond to pose
#         #     initialization requests sent to the /initialpose
#         #     topic. You can test that this works properly using the
#         #     "Pose Estimate" feature in RViz, which publishes to
#         #     /initialpose.

#         self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
#                                                  self.pose_callback,
#                                                  1)

#         # self.pose_estimate_sub = self.create_subscription(Odometry, "/pf/pose/odom",
#         #                                          self.average_particle_pose,
#         #                                          1)

#         #  *Important Note #3:* You must publish your pose estimate to
#         #     the following topic. In particular, you must use the
#         #     pose field of the Odometry message. You do not need to
#         #     provide the twist part of the Odometry message. The
#         #     odometry you publish here should be with respect to the
#         #     "/map" frame.

#         self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

#         self.pose_array_pub = self.create_publisher(PoseArray, "/particle_poses", 1)

#         # Initialize the models
#         self.motion_model = MotionModel(self)
#         self.sensor_model = SensorModel(self)
#         self.last_time = self.get_clock().now()

#         self.get_logger().info("=============+READY+=============")

#         # Implement the MCL algorithm
#         # using the sensor model and the motion model
#         #
#         # Make sure you include some way to initialize
#         # your particles, ideally with some sort
#         # of interactive interface in rviz
#         #
#         # Publish a transformation frame between the map
#         # and the particle_filter_frame.

#     def average_particle_pose(self):
#         avg_pose = np.average(self.particles[:, :2], axis = 0)

#         # theta, sins, cosines
#         thetas = self.particles[:,2]
#         sin_lis = np.average(np.sin(thetas), axis = 0)
#         cos_lis = np.average(np.cos(thetas), axis = 0)
#         avg_theta = np.arctan2(sin_lis, cos_lis)

#         ##################################
#         #           Odometry             #
#         ##################################
#         odom_msg = Odometry()
#         odom_msg.header.frame_id = "/map"
#         odom_msg.child_frame_id = self.particle_filter_frame

#         odom_msg.pose.pose.position.x = avg_pose[0]
#         odom_msg.pose.pose.position.y = avg_pose[1]
#         odom_msg.pose.pose.position.z = 0.0

#         # Using quaternions
#         quat = R.from_euler('xyz', [0.0, 0.0, avg_theta]).as_quat()
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # Publish the odometry message
#         self.odom_pub.publish(odom_msg)

#         ##################################
#         #         Transform Pose         #
#         ##################################
#         # transform pose should update based on previous position according to theta_k = theta_k-1 + delta_theta. in matrix form, Delta_x and Delta_y are given by 
#         # [Delta_x,Delta_y]^T = [[cos theta_k-1, sin theta_k-1],[-sin theta_k-1, cos theta_k-1]] [x_k-x_k-1, y_k-y_k-1]^T

#         transform_msg = TransformStamped()
#         transform_msg.header.frame_id = "/map"
#         transform_msg.child_frame_id = "/base_link_pf"

#         # x and y here should be Delta_x and Delta_y
#         (Delta_x, Delta_y) = np.dot([[np.cos(avg_theta), np.sin(avg_theta)],[-np.sin(avg_theta), np.cos(avg_theta)]], avg_pose - self.particles[0, :2])
#         transform_msg.transform.translation.x = Delta_x
#         transform_msg.transform.translation.y = Delta_y
#         transform_msg.transform.translation.z = 0.0

#         transform_msg.transform.rotation.x = quat[0]
#         transform_msg.transform.rotation.y = quat[1]
#         transform_msg.transform.rotation.z = quat[2]
#         transform_msg.transform.rotation.w = quat[3]

#         # Publish the transform message
#         self.tf_broadcaster.sendTransform(transform_msg)

#         ############################
#         # Pose Array
#         ############################
#         particles_msg = PoseArray()
#         poses = []

#         for particle in self.particles:
#             p = Pose()
#             p.position.x = float(particle[0])
#             p.position.y = float(particle[1])
#             p.position.z = 0.0

#             quat = R.from_euler('xyz', [0.0, 0.0, float(particle[2])]).as_quat()
#             p.orientation.x = quat[0]
#             p.orientation.y = quat[1]
#             p.orientation.z = quat[2]
#             p.orientation.w = quat[3]

#             poses.append(p)

#         particles_msg.header.frame_id = "/map"
#         # particles_msg.header = self.get_clock().now().to_msg()
#         particles_msg.poses = poses
#         self.pose_array_pub.publish(particles_msg)

#     def laser_callback(self, msg):
#         if self.particles is None:
#             return
#         ranges = np.array(msg.ranges)
#         probs = self.sensor_model.evaluate(self.particles, ranges)
#         probs = probs ** (1/3)
#         indices = np.random.choice(len(self.particles), self.num_particles, True, probs)
#         self.particles = self.particles[indices]

        


#     def odom_callback(self, msg):
#         if self.particles is None:
#             return
#         twist = msg.twist.twist
#         vx = twist.linear.x
#         vy = twist.linear.y
#         vyaw = twist.angular.z

#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time

#         dx = vx * dt
#         dy = vy * dt
#         dyaw = vyaw * dt
#         v = np.sqrt(vx**2 + vy**2)

#         # Derive steering angle from bicycle model inverse
#         L = self.motion_model.wheelbase
#         if v > 1e-3:
#             steering_angle = np.arctan(L * vyaw / v)
#         else:
#             steering_angle = 0.0

#         self.particles = self.motion_model.evaluate(
#             self.particles, (dx, dy, dyaw),
#             v=v, steering_angle=steering_angle, dt=dt
#         )

#     def pose_callback(self, msg):
#         pose = msg.pose.pose
#         x = pose.position.x
#         y = pose.position.y
#         quaternion = [pose.orientation.x, pose.orientation.y,
#                       pose.orientation.z, pose.orientation.w]
#         yaw = R.from_quat(quaternion).as_euler('xyz')[2]

#         mean = [x, y, yaw]

#         covariance = np.array(msg.pose.covariance).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
#         particles =  np.random.multivariate_normal(mean, covariance, self.num_particles)
#         self.particles = particles




# def main(args=None):
#     rclpy.init(args=args)
#     pf = ParticleFilter()
#     rclpy.spin(pf)
#     rclpy.shutdown()

# from localization.sensor_model import SensorModel
# from localization.motion_model import MotionModel

# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped

# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
# from sensor_msgs.msg import LaserScan


# from rclpy.node import Node
# import rclpy
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# import tf2_ros
# from tf2_ros import TransformBroadcaster

# assert rclpy


# class ParticleFilter(Node):

#     def __init__(self):
#         super().__init__("particle_filter")

#         self.declare_parameter('particle_filter_frame', "default")
#         self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

#         #  *Important Note #1:* It is critical for your particle
#         #     filter to obtain the following topic names from the
#         #     parameters for the autograder to work correctly. Note
#         #     that while the Odometry message contains both a pose and
#         #     a twist component, you will only be provided with the
#         #     twist component, so you should rely only on that
#         #     information, and *not* use the pose component.

#         # Initialize the transform broadcaster
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Initialize transform listener
#         self.buffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.buffer, self)

#         self.declare_parameter('odom_topic', "/odom")
#         self.declare_parameter('scan_topic', "/scan")

#         scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

#         self.num_particles = 200
#         self.laser_callback_freq = 20 #in Hz
#         self.laser_callback_prev_time = 0.0

#         self.particles = None

#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.laser_sub = self.create_subscription(LaserScan, scan_topic,
#                                                   self.laser_callback,
#                                                   1)

#         self.odom_sub = self.create_subscription(Odometry, odom_topic,
#                                                  self.odom_callback,
#                                                  1)

#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


#         #  *Important Note #2:* You must respond to pose
#         #     initialization requests sent to the /initialpose
#         #     topic. You can test that this works properly using the
#         #     "Pose Estimate" feature in RViz, which publishes to
#         #     /initialpose.

#         self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
#                                                  self.pose_callback,
#                                                  1)

#         # self.pose_estimate_sub = self.create_subscription(Odometry, "/pf/pose/odom",
#         #                                          self.average_particle_pose,
#         #                                          1)

#         #  *Important Note #3:* You must publish your pose estimate to
#         #     the following topic. In particular, you must use the
#         #     pose field of the Odometry message. You do not need to
#         #     provide the twist part of the Odometry message. The
#         #     odometry you publish here should be with respect to the
#         #     "/map" frame.

#         self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

#         self.pose_array_pub = self.create_publisher(PoseArray, "/particle_poses", 1)

#         # Initialize the models
#         self.motion_model = MotionModel(self)
#         self.sensor_model = SensorModel(self)
#         self.last_time = self.get_clock().now()

#         self.get_logger().info("=============+READY+=============")

#         # Implement the MCL algorithm
#         # using the sensor model and the motion model
#         #
#         # Make sure you include some way to initialize
#         # your particles, ideally with some sort
#         # of interactive interface in rviz
#         #
#         # Publish a transformation frame between the map
#         # and the particle_filter_frame.

#     def average_particle_pose(self):
#         avg_pose = np.average(self.particles[:, :2], axis = 0)

#         # theta, sins, cosines
#         thetas = self.particles[:,2]
#         sin_lis = np.average(np.sin(thetas), axis = 0)
#         cos_lis = np.average(np.cos(thetas), axis = 0)
#         avg_theta = np.arctan2(sin_lis, cos_lis)

#         ##################################
#         #           Odometry             #
#         ##################################
#         odom_msg = Odometry()
#         odom_msg.header.frame_id = "/map"
#         odom_msg.child_frame_id = self.particle_filter_frame

#         odom_msg.pose.pose.position.x = avg_pose[0]
#         odom_msg.pose.pose.position.y = avg_pose[1]
#         odom_msg.pose.pose.position.z = 0.0

#         # Using quaternions
#         quat = R.from_euler('xyz', [0.0, 0.0, avg_theta]).as_quat()
#         odom_msg.pose.pose.orientation.x = quat[0]
#         odom_msg.pose.pose.orientation.y = quat[1]
#         odom_msg.pose.pose.orientation.z = quat[2]
#         odom_msg.pose.pose.orientation.w = quat[3]

#         # Publish the odometry message
#         self.odom_pub.publish(odom_msg)

#         ##################################
#         #         Transform Pose         #
#         ##################################
#         # transform pose should update based on previous position according to theta_k = theta_k-1 + delta_theta. in matrix form, Delta_x and Delta_y are given by
#         # [Delta_x,Delta_y]^T = [[cos theta_k-1, sin theta_k-1],[-sin theta_k-1, cos theta_k-1]] [x_k-x_k-1, y_k-y_k-1]^T

#         transform_msg = TransformStamped()
#         transform_msg.header.frame_id = "/map"
#         transform_msg.child_frame_id = "/base_link_pf"

#         # x and y here should be Delta_x and Delta_y
#         (Delta_x, Delta_y) = np.dot([[np.cos(avg_theta), np.sin(avg_theta)],[-np.sin(avg_theta), np.cos(avg_theta)]], avg_pose - self.particles[0, :2])
#         transform_msg.transform.translation.x = Delta_x
#         transform_msg.transform.translation.y = Delta_y
#         transform_msg.transform.translation.z = 0.0

#         transform_msg.transform.rotation.x = quat[0]
#         transform_msg.transform.rotation.y = quat[1]
#         transform_msg.transform.rotation.z = quat[2]
#         transform_msg.transform.rotation.w = quat[3]

#         # Publish the transform message
#         self.tf_broadcaster.sendTransform(transform_msg)

#         ############################
#         # Pose Array
#         ############################
#         particles_msg = PoseArray()
#         poses = []

#         for particle in self.particles:
#             p = Pose()
#             p.position.x = float(particle[0])
#             p.position.y = float(particle[1])
#             p.position.z = 0.0

#             quat = R.from_euler('xyz', [0.0, 0.0, float(particle[2])]).as_quat()
#             p.orientation.x = quat[0]
#             p.orientation.y = quat[1]
#             p.orientation.z = quat[2]
#             p.orientation.w = quat[3]

#             poses.append(p)

#         particles_msg.header.frame_id = "/map"
#         # particles_msg.header = self.get_clock().now().to_msg()
#         particles_msg.poses = poses
#         self.pose_array_pub.publish(particles_msg)

#     def laser_callback(self, msg):
#         if self.particles is None:
#             return
#         ranges = np.array(msg.ranges)
#         probs = self.sensor_model.evaluate(self.particles, ranges)
#         probs = probs ** (1/3)
#         indices = np.random.choice(len(self.particles), self.num_particles, True, probs)
#         self.particles = self.particles[indices]

#     def odom_callback(self, msg):
#         if self.particles is None:
#             return
#         twist = msg.twist.twist
#         vx = twist.linear.x
#         vy = twist.linear.y
#         vyaw = twist.angular.z

#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time

#         dx = vx * dt
#         dy = vy * dt
#         dyaw = vyaw * dt
#         v = np.sqrt(vx**2 + vy**2)

#         # Derive steering angle from bicycle model inverse
#         L = self.motion_model.wheelbase
#         if v > 1e-3:
#             steering_angle = np.arctan(L * vyaw / v)
#         else:
#             steering_angle = 0.0

#         self.particles = self.motion_model.evaluate(
#             self.particles, (dx, dy, dyaw),
#             v=v, steering_angle=steering_angle, dt=dt
#         )

#     def pose_callback(self, msg):
#         pose = msg.pose.pose
#         x = pose.position.x
#         y = pose.position.y
#         quaternion = [pose.orientation.x, pose.orientation.y,
#                       pose.orientation.z, pose.orientation.w]
#         yaw = R.from_quat(quaternion).as_euler('xyz')[2]

#         mean = [x, y, yaw]

#         covariance = np.array(msg.pose.covariance).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
#         particles =  np.random.multivariate_normal(mean, covariance, self.num_particles)
#         self.particles = particles

# def main(args=None):
#     rclpy.init(args=args)
#     pf = ParticleFilter()
#     rclpy.spin(pf)
#     rclpy.shutdown()