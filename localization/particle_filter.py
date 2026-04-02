from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from sensor_msgs.msg import LaserScan


from rclpy.node import Node
import rclpy
import numpy as np


assert rclpy


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.

        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('scan_topic', "/scan")

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.num_particles = 200
        self.laser_callback_freq = 20 #in Hz
        self.laser_callback_prev_time = 0.0

        self.particles = None

        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        # self.pose_estimate_sub = self.create_subscription(Odometry, odo,
        #                                          self.average_particle_pose,
        #                                          1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

        self.pose_pub = self.create_publisher(PoseArray, "/particles", 1)
        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)
        self.last_time = self.get_clock().now()

        self.get_logger().info("=============+READY+=============")

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

    def laser_callback(self, msg):
        if self.particles is None:
            return
        ranges = np.array(msg.ranges)
        probs = self.sensor_model.evaluate(self.particles, ranges)
        # self.get_logger().info(f"Probabilities: {probs}")
        if probs =="Map not set":
            return
        indices = np.random.choice(len(self.particles), self.num_particles, True, probs/np.sum(probs))
        self.particles = self.particles[indices]


    def odom_callback(self, msg):
        if self.particles is None:
            return
        twist = msg.twist.twist
        vx = twist.linear.x
        vy = twist.linear.y
        vyaw = twist.angular.z

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        dx = vx * dt
        dy = vy * dt
        dyaw = vyaw * dt
        v = np.sqrt(vx**2 + vy**2)

        # Derive steering angle from bicycle model inverse
        L = self.motion_model.wheelbase
        if v > 1e-3:
            steering_angle = np.arctan(L * vyaw / v)
        else:
            steering_angle = 0.0

        self.particles = self.motion_model.evaluate(
            self.particles, (dx, dy, dyaw),
            v=v, steering_angle=steering_angle, dt=dt
        )

        avg_pose = np.average(self.particles[:, :2], axis = 0)

        # theta, sins, cosines
        thetas = self.particles[:,2]
        sin_lis = np.average(np.sin(thetas), axis = 0)
        cos_lis = np.average(np.cos(thetas), axis = 0)
        avg_theta = np.arctan2(sin_lis, cos_lis)

        ##################################
        #           Odometry             #
        ##################################
        odom_msg = Odometry()
        odom_msg.header.frame_id = "/map"
        odom_msg.child_frame_id = self.particle_filter_frame

        odom_msg.pose.pose.position.x = avg_pose[0]
        odom_msg.pose.pose.position.y = avg_pose[1]
        odom_msg.pose.pose.position.z = 0.0

        # Using quaternions
        quat = quaternion_from_euler(0, 0, avg_theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        self.get_logger().info(f"Publishing {avg_pose[0]}, {avg_pose[1]}")
        # Publish the odometry message
        self.odom_pub.publish(odom_msg)



        ##################################
        #         Transform Pose         #
        ##################################
        transform_msg = TransformStamped()
        transform_msg.header.frame_id = "/map"
        transform_msg.child_frame_id = "/base_link_pf"

        transform_msg.transform.translation.x = avg_pose[0]
        transform_msg.transform.translation.y = avg_pose[1]
        transform_msg.transform.translation.z = 0.0

        transform_msg.transform.rotation.x = quat[0]
        transform_msg.transform.rotation.y = quat[1]
        transform_msg.transform.rotation.z = quat[2]
        transform_msg.transform.rotation.w = quat[3]

        # Publish the transform message
        self.tf_broadcaster.sendTransform(transform_msg)

        particle_msg = PoseArray()
        particle_msg.header.frame_id = "/map"
        particle_msg.header.stamp = self.get_clock().now().to_msg()
        particle_msg.poses = []
        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle[0]
            pose.position.y = particle[1]
            pose.orientation.z = np.sin(particle[2]/2)
            pose.orientation.w = np.cos(particle[2]/2)
            particle_msg.poses.append(pose)
        self.pose_pub.publish(particle_msg)





    def pose_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)

        mean = [x, y, yaw]

        covariance = np.array(msg.pose.covariance).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
        particles =  np.random.multivariate_normal(mean, covariance, self.num_particles)
        self.particles = particles











def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
