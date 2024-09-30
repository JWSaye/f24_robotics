#!/usr/bin/env python3

import math
import numpy
import sys
import termios

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

from turtlebot3_example.turtlebot3_position_control.turtlebot3_path import Turtlebot3Path


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.goal_pose_x = [1.0, 1.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_pose_theta = [0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 180.0, 180.0, 360.0, 360.0]
        self.lin_vel = [0.075, 0.150, 0.075, 0.150, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # m/s
        self.ang_vel = [0.0, 0.0, 0.0, 0.0, 0.52, 2.094, 0.52, 2.094, 0.52, 2.094] # rad/s
        self.step = 1
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
        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state is True:
            self.generate_path()

    def generate_path(self):
        twist = Twist()
        
        # Step 1: Turn
        if self.step == 1:
            path_theta = math.atan2(
                self.goal_pose_y - self.last_pose_y,
                self.goal_pose_x - self.last_pose_x)
            angle = path_theta - self.last_pose_theta
            angular_velocity = 0.1  # unit: rad/s
            twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

        # Step 2: Go Straight
        elif self.step == 2:
            linear_velocity = 0.1  # unit: m/s
            twist, self.step = Turtlebot3Path.go_straight(self.goal_pose_x, linear_velocity, self.step)

            self.cmd_vel_pub.publish(twist)

    def get_key(self):
        # Print terminal message and get inputs
        print(terminal_msg)
        input_x = float(input("Input x: "))
        input_y = float(input("Input y: "))
        input_theta = float(input("Input theta: "))
        while input_theta > 180 or input_theta < -180:
            self.get_logger().info("Enter a value for theta between -180 and 180")
            input_theta = input("Input theta: ")
        input_theta = numpy.deg2rad(input_theta)  # Convert [deg] to [rad]

        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_x, input_y, input_theta