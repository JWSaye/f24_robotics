#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
import math
from turtlebot3_example.turtlebot3_position_control.turtlebot3_path import Turtlebot3Path


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = [1.0, 1.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_pose_theta = [0.0, 0.0, 0.0, 0.0, 0.17453, 0.17453, 1.0, 1.0, 2.0, 2.0]
        self.lin_vel = [0.075, 0.150, 0.075, 0.150, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # m/s
        self.ang_vel = [0.0, 0.0, 0.0, 0.0, 0.52, 2.094, 0.52, 2.094, 0.52, 2.094] # rad/s
        self.step = 8
        self.get_key_state = False
        self.init_odom_state = False  # To get the initial pose at the beginning

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(0.010, self.update_callback)  # unit: s

        self.get_logger().info("Turtlebot3 position control node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_theta = msg.pose.pose.orientation.z
        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state is True:
            self.generate_path()

    def generate_path(self):
        twist = Twist()
        
        # Step 1: Turn
        if self.step > 3:
            angle = self.goal_pose_theta[self.step] - self.last_pose_theta
            twist = Twist()

            if math.fabs(angle) > 0.01:  # 0.01 is small enough value
                if angle >= math.pi:
                    twist.angular.z = -0.52
                elif math.pi > angle and angle >= 0:
                    twist.angular.z = 0.52
                elif 0 > angle and angle >= -math.pi:
                    twist.angular.z = -0.52
                elif angle > -math.pi:
                    twist.angular.z = 0.52
                self.get_logger().info("Orientation Z: %f" % self.last_pose_theta)

        # Step 2: Go Straight
        elif self.step <= 3:
            path = 1 - self.last_pose_x
            twist = Twist()

            if path > 0.0005:  # 0.01 is small enough value
                twist.linear.x = 0.075
                self.get_logger().info("Pose X: %f" % self.last_pose_x)

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_position_control = Turtlebot3PositionControl()
    rclpy.spin(turtlebot3_position_control)
    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()