from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion

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

        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)
        self.last_time = None

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

        

        def laser_callback(msg):
            ranges = np.array(msg.ranges)
             = self.sensor_model.evaluate(self.particles, ranges)

        def odom_callback(msg):
            pass
            twist = msg.twist.twist
            vx = twist.linear.x
            vy = twist.linear.y
            vyaw = twist.angular.z

            if self.last_time is None:
                self.last_time = rclpy.time.Time(msg.header.stamp)
            else:
                current_time = rclpy.time.Time(msg.header.stamp)
                dt = (current_time - self.last_time).nanoseconds / 1e9

            dx = vx * dt
            dy = vy * dt
            dyaw = vyaw * dt

            self.particles = self.motion_model.evaluate(self.particles, (dx, dy, dyaw))



        def pose_callback(msg):
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            quaternion = (pose.orientation.x, pose.orientation.y, 
                        pose.orientation.z, pose.orientation.w)
            _, _, yaw = euler_from_quaternion(quaternion)
            
            mean = [x, y, yaw]

            covariance = np.array(msg.pose.covariance).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
            # Change this (100) to number of particle variable
            particles =  np.random.multivariate_normal(mean, covariance, 100)
            self.particles = particles



def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
