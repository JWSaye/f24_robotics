# import the ROS2 API python libraries
import rclpy
# import the ROS2 Node API python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import the Odometry module from nav_msgs interface
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

import math
from   enum import Enum, auto
import numpy
import os
import datetime

class RunModes(Enum):
    # webots run mode
    MODE_WEBOTS = auto()
    # inverse run mode
    MODE_INVERSE = auto()
    # mirror run mode
    MODE_MIRROR  = auto()

# configures which mode we are in
MODE = RunModes.MODE_WEBOTS

if MODE != RunModes.MODE_WEBOTS:
    from apriltag_msgs.msg import AprilTagDetectionArray

# the number of duplicate positions before entering error mode
MAX_DUP_POSITIONS                  = 15

# the amount of radians to turn when scanning for april tags
DEGREE_TO_TURN                     = 6.28319  # 360 degrees

# rotate speed configuration
ROTATE_FORWARD_SPEED               = 0.0
ROTATE_ANGULAR_SPEED               = 0.6

# the distance to travel before completing a rotation
ROTATE_DISTANCE                    = 7

# distance at which to detect walls and objects
FORWARD_DETECT_RANGE               = 1.0
LEFT_DETECT_RANGE                  = 0.0
RIGHT_DETECT_RANGE                 = 1.5
BACKWARD_DETECT_RANGE              = 0.0

# forward speed configuration
NORMAL_FORWARD_LINEAR_SPEED        = 0.125
NORMAL_FORWARD_ANGULAR_SPEED       = 0.0

# turning left speed configuration
NORMAL_TURNING_LEFT_LINEAR_SPEED   = 0.075
NORMAL_TURNING_LEFT_ANGULAR_SPEED  = 0.6

# turning right speed configuration
NORMAL_TURNING_RIGHT_LINEAR_SPEED  = 0.1
NORMAL_TURNING_RIGHT_ANGULAR_SPEED = -0.25

# backward speed configuration
NORMAL_BACKWARD_LINEAR_SPEED       = -0.075
NORMAL_BACKWARD_ANGULAR_SPEED      = 0.0

# struck speed configuration
STUCK_LINEAR_SPEED                 = 0.0
STUCK_ANGULAR_SPEED                = 0.0

# error detection forward speed configuration
ERROR_FORWARD_LINEAR_SPEED         = 0.1
ERROR_FORWARD_ANGULAR_SPEED        = 0.0

# error detection left speed configuration
ERROR_LEFT_LINEAR_SPEED           = 0.075
ERROR_LEFT_ANGULAR_SPEED          = 0.5

# error detection backward speed configuration
ERROR_BACKWARD_LINEAR_SPEED       = -0.1
ERROR_BACKWARD_ANGULAR_SPEED      = 0.0

# webots indexes
WEBOTS_INITIAL_BACK_INDEX = 0
WEBOTS_BACK_LEFT_INDEX    = 45
WEBOTS_LEFT_INDEX         = 90
WEBOTS_FRONT_LEFT_INDEX   = 135
WEBOTS_FRONT_INDEX        = 180
WEBOTS_FRONT_RIGHT_INDEX  = 225
WEBOTS_RIGHT_INDEX        = 270
WEBOTS_BACK_RIGHT_INDEX   = 315
WEBOTS_FINAL_BACK_INDEX   = 359

# real world inversed indexes
INVERSE_FRONT_LEFT_INDEX       = 315
INVERSE_FRONT_LEFT_STOP_INDEX  = 359
INVERSE_FRONT_RIGHT_INDEX      = 0
INVERSE_FRONT_RIGHT_STOP_INDEX = 45
INVERSE_RIGHT_INDEX            = 90
INVERSE_BACK_RIGHT_INDEX       = 135
INVERSE_BACK_INDEX             = 180
INVERSE_BACK_LEFT_INDEX        = 225
INVERSE_LEFT_INDEX             = 270

# real world mirrored indexes
MIRROR_FRONT_INDEX           = 180
MIRROR_FRONT_RIGHT_INDEX     = 135
MIRROR_RIGHT_INDEX           = 90
MIRROR_BACK_LEFT_INDEX       = 315
MIRROR_BACK_LEFT_STOP_INDEX  = 359
MIRROR_BACK_RIGHT_INDEX      = 0
MIRROR_BACK_RIGHT_STOP_INDEX = 45
MIRROR_LEFT_INDEX            = 270
MIRROR_FRONT_LEFT_INDEX      = 225

FOUND_TAGS = []

class WallFollowingStates(Enum):
    # initial wall finding state
    STATE_WALL_FINDER   = auto()
    # nothing is detected, following wall, or moving past object
    STATE_MOVE_FORWARD  = auto()
    # turning around a corner due to wall or object
    STATE_TURN_RIGHT    = auto()
    # reached a corner or moving around an object
    STATE_TURN_LEFT     = auto()
    # reached a dead end only option is to go backward
    STATE_MOVE_BACKWARD = auto()
    # attempted to go backward unsuccessfully, robot is stuck
    STATE_STUCK         = auto()
    # state that rotates the turtlebot 360 degrees to search for april tags
    STATE_ROTATE        = auto()

class RunModes(Enum):
    # webots run mode
    MODE_WEBOTS = auto()
    # inverse run mode
    MODE_INVERSE = auto()
    # mirror run mode
    MODE_MIRROR  = auto()

# configures which mode we are in
MODE = RunModes.MODE_WEBOTS

class WallFollower(Node):
    '''
    Wall following Ros node.
    '''

    def __init__(self):
        '''
        The initializing function of the wall follower node.
        '''
        # initialize the publisher via parent class
        super().__init__('wall_follower_node')

        # stores most up-to-date odometry and laser scan data
        self.scan_cleaned  = []
        self.pose_saved    = ''
        self.orient_saved  = ''

        # stores the x and y distance the turtlebot has traveld since last reset
        self.x_distance_traveled = 0.0
        self.y_distance_traveled = 0.0
        # stores the radians turned by the turtlebot since last reset
        self.radians_turned      = 0.0

        # variables used to initiate and handle the error recovery state
        self.duplicate_state_count = 0
        self.recovery_cycles       = 0

        # stores turtlebot state status
        self.state          = WallFollowingStates.STATE_WALL_FINDER
        self.previous_state = self.state

        # will be used to send commands to the robot
        self.cmd = Twist()

        # initialize the publisher for velcity commands
        self.publisher_  = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # subscription to lidar data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # initialize the subscription to odometry data
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )


        if MODE != RunModes.MODE_WEBOTS:
            # Initialize the AprilTag subscriber
            self.apriltag_subscription = self.create_subscription(
                AprilTagDetectionArray,
                '/detections',
                self.apriltag_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            )
            # Set up logging for AprilTag detections
            self.log_file = open("apriltag_detections.log", "a")
            # Launch AprilTag nodes
            self.launch_apriltag_nodes()

        # amount of time between checks for updates
        timer_period = 0.5
        # the time that will cause the callback to execute
        self.timer   = self.create_timer(timer_period, self.timer_callback)

    def launch_apriltag_nodes(self):
        # Start the camera and AprilTag node
        os.system("ros2 run v4l2_camera v4l2_camera_node &")
        os.system("ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file /opt/ros/humble/share/apriltag_ros/cfg/tags_36h11.yaml &")

    def apriltag_callback(self, msg):
        # Callback for processing AprilTag detections
        for detection in msg.detections:
            if detection.id not in FOUND_TAGS:
                tag_id = detection.id  # Assuming a single tag per detection
                FOUND_TAGS.append(tag_id)
                position = detection.centre

            	# Log the detection
                log_entry = (f"{datetime.datetime.now()} - Detected Tag ID: {tag_id}, Position: (x: {position.x}, y: {position.y}")
                self.log_file.write(log_entry)
                self.log_file.flush()
                self.get_logger().info(log_entry)

    def destroy_node(self):
        # Close the log file when shutting down
        self.log_file.close()
        super().destroy_node()

    def lidar_listener_callback(self, lidar_msg):
        '''
        Callback that executes upon new LaserScan data being received.
        Parameters:
        -----------
            lidar_msg: The msg received via the LaserScan subscription.
        '''
        # erase stale lidar data
        self.scan_cleaned = []
        # pare the lidar data from the message
        lidar_scan = lidar_msg.ranges

        # clean lidar scan data
        for reading in lidar_scan:
            if MODE == RunModes.MODE_WEBOTS:
                if float('Inf') == reading:
                    self.scan_cleaned.append(3.5)
                elif math.isnan(reading):
                    self.scan_cleaned.append(0.0)
                else:
                    self.scan_cleaned.append(reading)
            else: # Inverse or Mirror
                if float('Inf') == reading:
                    self.scan_cleaned.append(3.5)
                elif math.isnan(reading):
                    self.scan_cleaned.append(3.5)
                elif reading > 3.5:
                    self.scan_cleaned.append(3.5)
                elif reading == 0:
                    self.scan_cleaned.append(3.5)
                else:
                    self.scan_cleaned.append(reading)

        return

    def odometry_listener_callback(self, odom_msg):
        '''
        Callback that is executed when new odomoetry data is available.
        Parameters:
        -----------
            odom_msg: The new odometry position data.
        '''
        # get the current position as reported by the turtlebot
        position    = odom_msg.pose.pose.position
        # get the current orientation as reported by the turtlebot
        orientation = odom_msg.pose.pose.orientation

        # store the current position and orientation data
        (posx, posy)     = (position.x, position.y)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Compute distance traveled and degrees turned
        if self.pose_saved != '' and self.orient_saved != '':

            # Get the euler position in
            saved_roll, saved_pitch, saved_yaw = self.euler_from_quaternion(self.orient_saved)
            new_roll, new_pitch, new_yaw       = self.euler_from_quaternion(orientation)

            self.x_distance_traveled += abs(posx - self.pose_saved.x)
            self.y_distance_traveled += abs(posy - self.pose_saved.y)

            # Determine the radians turned
            self.radians_turned += abs(abs(new_yaw) - abs(saved_yaw))

        # similarly for twist message if you need
        self.pose_saved   = position
        self.orient_saved = orientation

        return

    def euler_from_quaternion(self, quat):
        '''
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        NOTE: This code was taken from the turtlebot3 position example.

        Parameters:
        -----------
            quat: The orientation in quaternion form [x, y, z, w].

        Returns:
            x: The x orientation in euler form (roll in radians)
            y: The y orientation in euler form (pitch in radians)
            z: The z orientation in euler form (yaw in radians)
        '''
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def object_detected(self, min_foward_lidar, min_left_lidar, min_right_lidar, min_backward_lidar):
        '''
        Determines if an object is detected within the specified range or not.
        '''
        b_forward_object  = min_foward_lidar   < FORWARD_DETECT_RANGE
        b_left_object     = min_left_lidar     < LEFT_DETECT_RANGE
        b_right_object    = min_right_lidar    < RIGHT_DETECT_RANGE
        b_backward_object = min_backward_lidar < BACKWARD_DETECT_RANGE

        return (b_forward_object, b_left_object, b_right_object, b_backward_object)

    def get_lidar_data(self):
        '''
        Returns the lidar data associated with the mode we are running in.

        Returns:
        --------
            left_lidar_data
            right_lidar_data
            front_lidar_data
            back_lidar_data
        '''
        left_lidar_data  = []
        right_lidar_data = []
        front_lidar_data = []
        back_lidar_data  = []

        if MODE == RunModes.MODE_WEBOTS:
            left_lidar_data       = self.scan_cleaned[WEBOTS_BACK_LEFT_INDEX:WEBOTS_FRONT_LEFT_INDEX]
            right_lidar_data      = self.scan_cleaned[WEBOTS_FRONT_RIGHT_INDEX:WEBOTS_BACK_RIGHT_INDEX]
            front_lidar_data      = self.scan_cleaned[WEBOTS_FRONT_LEFT_INDEX:WEBOTS_FRONT_RIGHT_INDEX]
            back_left_lidar_data  = self.scan_cleaned[WEBOTS_INITIAL_BACK_INDEX:WEBOTS_BACK_LEFT_INDEX]
            back_right_lidar_data = self.scan_cleaned[WEBOTS_BACK_RIGHT_INDEX:WEBOTS_FINAL_BACK_INDEX]
            back_lidar_data       = back_left_lidar_data + back_right_lidar_data

        elif MODE == RunModes.MODE_INVERSE:
            left_lidar_data        = self.scan_cleaned[INVERSE_BACK_LEFT_INDEX:INVERSE_FRONT_LEFT_INDEX]
            right_lidar_data       = self.scan_cleaned[INVERSE_FRONT_RIGHT_STOP_INDEX:INVERSE_BACK_RIGHT_INDEX]
            front_left_lidar_data  = self.scan_cleaned[INVERSE_FRONT_LEFT_INDEX:INVERSE_FRONT_LEFT_STOP_INDEX]
            front_right_lidar_data = self.scan_cleaned[INVERSE_FRONT_RIGHT_INDEX:INVERSE_FRONT_RIGHT_STOP_INDEX]
            front_lidar_data       = front_left_lidar_data + front_right_lidar_data
            back_lidar_data        = self.scan_cleaned[INVERSE_BACK_RIGHT_INDEX:INVERSE_BACK_LEFT_INDEX]

        elif MODE == RunModes.MODE_MIRROR:
            left_lidar_data       = self.scan_cleaned[MIRROR_FRONT_LEFT_INDEX:MIRROR_BACK_LEFT_INDEX]
            right_lidar_data      = self.scan_cleaned[MIRROR_BACK_RIGHT_STOP_INDEX:MIRROR_FRONT_RIGHT_INDEX]
            front_lidar_data      = self.scan_cleaned[MIRROR_FRONT_RIGHT_INDEX:MIRROR_FRONT_LEFT_INDEX]
            back_left_lidar_data  = self.scan_cleaned[MIRROR_BACK_LEFT_INDEX:MIRROR_BACK_LEFT_STOP_INDEX]
            back_right_lidar_data = self.scan_cleaned[MIRROR_BACK_RIGHT_INDEX:MIRROR_BACK_RIGHT_STOP_INDEX]
            back_lidar_data       = back_left_lidar_data + back_right_lidar_data

        return left_lidar_data, right_lidar_data, front_lidar_data, back_lidar_data

    def get_state(self):
        '''
        Determines what state the robot is in based on the lidar data.

        @note This function assumes the lidar data stored in instance variable
              is up to date and accurate.

        Returns:
        --------
            WallFollowingStates:
                An enum variable representing which state that the robot is
                currently in.
        '''
        # Get the lidar data for the different regions
        left_lidar_data, right_lidar_data, front_lidar_data, back_lidar_data = self.get_lidar_data()

        right_lidar_data = [i for i in right_lidar_data if i != 0.0]
        left_lidar_data  = [i for i in left_lidar_data if i != 0.0]
        back_lidar_data  = [i for i in back_lidar_data if i != 0.0]
        front_lidar_data = [i for i in front_lidar_data if i != 0.0]

        print("Back Lidar Data:")
        print(str(back_lidar_data))
        print("Front Lidar Data:")
        print(str(front_lidar_data))
        print("Right Lidar Data:")
        print(str(back_lidar_data))
        print("Left Lidar Data:")
        print(str(back_lidar_data))

        # Get the minimum data to determine robot's state
        min_left_lidar_data  = min(left_lidar_data)
        min_right_lidar_data = min(right_lidar_data)
        min_front_lidar_data = min(front_lidar_data)
        min_back_lidar_data  = min(back_lidar_data)

        # Determine where objects are detected
        b_forward_object, b_left_object, b_right_object, b_backward_object = self.object_detected(
            min_front_lidar_data,
            min_left_lidar_data,
            min_right_lidar_data,
            min_back_lidar_data
        )

        # Obstacle in all regions
        if b_left_object and b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in all regions')
            return WallFollowingStates.STATE_STUCK

        # Obstacle in no regions
        if not b_left_object and not b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in no regions')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in left, right, and front regions
        elif b_left_object and b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in left, right, and front regions')
            return WallFollowingStates.STATE_MOVE_BACKWARD

        # Obstacle in left, right, and back regions
        elif b_left_object and b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in left, right, and back regions')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in right, front, and back regions
        elif not b_left_object and b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in right, front, and back regions')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in left, front, and back regions
        elif b_left_object and not b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in left, front, and back regions')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in left and right region
        elif b_left_object and b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in left and right regions')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in front and right region
        elif not b_left_object and b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in front and right regions')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in front and left region
        elif b_left_object and not b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in front and left regions')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in front and back region
        elif not b_left_object and not b_right_object and b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in front and back regions')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in back and left region
        elif b_left_object and not b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in back and left region')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in back and right region
        elif not b_left_object and b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in back and right region')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in front region only
        elif not b_left_object and not b_right_object and b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in front region')
            return WallFollowingStates.STATE_TURN_LEFT

        # Obstacle in back region only
        elif not b_left_object and not b_right_object and not b_forward_object and b_backward_object:
            self.get_logger().info('obstacle detected in back region')
            return WallFollowingStates.STATE_MOVE_FORWARD

        # Obstacle in right region only
        elif not b_left_object and b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in right region')
            return WallFollowingStates.STATE_TURN_RIGHT

        # Obstacle in left region only
        elif b_left_object and not b_right_object and not b_forward_object and not b_backward_object:
            self.get_logger().info('obstacle detected in left region')
            return WallFollowingStates.STATE_TURN_RIGHT

        return WallFollowingStates.STATE_STUCK

    def wall_detected(self):
        '''
        Returns whether a wall is detected in front or to the right of the
        turtlebot.
        '''
        wall_detected = False

        # Get the lidar data for the different regions
        left_lidar_data, right_lidar_data, front_lidar_data, back_lidar_data = self.get_lidar_data()

        max_front_lidar_data  = max(front_lidar_data)

        print(str(front_lidar_data))

        # if the maximum lidar data is the max lidar range, there is an opening
        # signaling a wall has not yet been found
        if max_front_lidar_data <= 1:
            wall_detected = True

        return wall_detected

    def wall_finder_mode(self):
        '''
        Initial mode that will run until the turtlebot detects a wall in the
        front or right side of the turtlebot.
        '''
        self.cmd.linear.x  = NORMAL_FORWARD_LINEAR_SPEED
        self.cmd.angular.z = NORMAL_FORWARD_ANGULAR_SPEED

        self.publisher_.publish(self.cmd)

        # once a wall is detected, this mode should be exited and never run
        # again
        if self.wall_detected():
            self.state = WallFollowingStates.STATE_ROTATE

        return

    def rotate_mode(self):
        '''
        Rotates the specifed number of degrees before returning to wall following.
        '''
        if (self.x_distance_traveled + self.y_distance_traveled) > ROTATE_DISTANCE:
            self.x_distance_traveled = 0.0
            self.y_distance_traveled = 0.0
            self.radians_turned      = 0.0

        self.cmd.linear.x  = ROTATE_FORWARD_SPEED
        self.cmd.angular.z = ROTATE_ANGULAR_SPEED

        self.publisher_.publish(self.cmd)

        if (self.radians_turned >= DEGREE_TO_TURN):
            self.x_distance_traveled = 0.0
            self.y_distance_traveled = 0.0
            self.radians_turned      = 0.0
            self.state = WallFollowingStates.STATE_MOVE_FORWARD

        return

    def wall_follower_mode(self):
        '''
        '''
        # determine which state we are in
        self.previous_state = self.state
        self.state          = self.get_state()

        if self.previous_state == self.state:

            self.get_logger().info('Duplicate State Detected')
            self.duplicate_state_count += 1

        else:

            self.get_logger().info('New State Detected')
            self.duplicate_state_count = 0

            # only check rotation status if we are sure the turtlebot is not stuck
            if (self.x_distance_traveled + self.y_distance_traveled) >= ROTATE_DISTANCE:
                self.state = WallFollowingStates.STATE_ROTATE

        if WallFollowingStates.STATE_MOVE_FORWARD == self.state:
            self.get_logger().info('Entering Move Forward State')
            self.cmd.linear.x  = NORMAL_FORWARD_LINEAR_SPEED
            self.cmd.angular.z = NORMAL_FORWARD_ANGULAR_SPEED

        elif WallFollowingStates.STATE_MOVE_BACKWARD == self.state:
            self.get_logger().info('Entering Move Backward State')
            self.cmd.linear.x  = NORMAL_BACKWARD_LINEAR_SPEED
            self.cmd.angular.z = NORMAL_BACKWARD_ANGULAR_SPEED

        elif WallFollowingStates.STATE_TURN_RIGHT == self.state:
            self.get_logger().info('Entering Turn Right State')
            self.cmd.linear.x  = NORMAL_TURNING_RIGHT_LINEAR_SPEED
            self.cmd.angular.z = NORMAL_TURNING_RIGHT_ANGULAR_SPEED

        elif WallFollowingStates.STATE_TURN_LEFT == self.state:
            self.get_logger().info('Entering Turn Left State')
            self.cmd.linear.x  = NORMAL_TURNING_LEFT_LINEAR_SPEED
            self.cmd.angular.z = NORMAL_TURNING_LEFT_ANGULAR_SPEED

        elif WallFollowingStates.STATE_STUCK == self.state:
            self.get_logger().info('Entering Stuck State')
            self.cmd.linear.x  = STUCK_LINEAR_SPEED
            self.cmd.angular.z = STUCK_ANGULAR_SPEED

        else:
            self.get_logger().info('Invalid State Detected!')
            self.cmd.linear.x  = STUCK_LINEAR_SPEED
            self.cmd.angular.z = STUCK_ANGULAR_SPEED

        self.publisher_.publish(self.cmd)

        return

    def error_recovery_mode(self):
        '''
        '''
        # move backward in attempt gain ability to turn
        if self.recovery_cycles < 5:
            self.get_logger().info('Move Backward to Attempt to Free Robot')
            self.cmd.linear.x  = ERROR_BACKWARD_LINEAR_SPEED
            self.cmd.angular.z = ERROR_BACKWARD_ANGULAR_SPEED
            self.publisher_.publish(self.cmd)

        # attempt to turn around object we are stuck at
        elif self.recovery_cycles < 10:
            self.get_logger().info('Turn Left to Attempt to Free Robot')
            self.cmd.linear.x  = ERROR_LEFT_LINEAR_SPEED
            self.cmd.angular.z = ERROR_LEFT_ANGULAR_SPEED
            self.publisher_.publish(self.cmd)

        # attempt to move forward past the object we are stuck at
        elif self.recovery_cycles < 15:
            self.get_logger().info('Move Forward to Attempt to Free Robot')
            self.cmd.linear.x  = ERROR_FORWARD_LINEAR_SPEED
            self.cmd.angular.z = ERROR_FORWARD_ANGULAR_SPEED
            self.publisher_.publish(self.cmd)

        # return to normal state machine
        else:
            self.get_logger().info('Exiting Recovery State')
            self.duplicate_state_count   = 0
            self.recovery_cycles         = 0

        self.recovery_cycles += 1

        return

    def timer_callback(self):
        '''
        Callback function that will execute once during the specified time
        period. This will collect the laser and odometry data before calling
        the current state's function.
        '''
        # if we have not yet received any lidar data exit immediantely
        if (len(self.scan_cleaned) == 0) or (self.pose_saved == '') or (self.orient_saved == ''):
            return

        # enter wall finder mode
        if WallFollowingStates.STATE_WALL_FINDER == self.state:
            self.get_logger().info('running wall finder mode')
            self.wall_finder_mode()

        # enter rotation mode
        elif WallFollowingStates.STATE_ROTATE == self.state:
            self.get_logger().info('running rotate mode')
            self.rotate_mode()

        # enter wall following mode
        elif MAX_DUP_POSITIONS >= self.duplicate_state_count:

            self.get_logger().info('running wall follower mode')
            self.wall_follower_mode()

        # enter error recovery mode
        else:

            self.get_logger().info('running error recovery mode')
            self.error_recovery_mode()

        return

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_follower_node = WallFollower()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_follower_node)
    # Explicity destroy the node
    wall_follower_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
