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

# the number of duplicate positions before entering error mode
MAX_DUP_POSITIONS                  = 30

# distance at which to detect walls and objects
FORWARD_DETECT_RANGE               = 0.6
LEFT_DETECT_RANGE                  = 0.0
RIGHT_DETECT_RANGE                 = 0.7
BACKWARD_DETECT_RANGE              = 0.0

# forward speed configuration
NORMAL_FORWARD_LINEAR_SPEED        = 0.075
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

# The indexes of various position in the laser scan array
INITIAL_BACK_INDEX = 0
BACK_LEFT_INDEX    = 45
LEFT_INDEX         = 90
FRONT_LEFT_INDEX   = 135
FRONT_INDEX        = 180
FRONT_RIGHT_INDEX  = 225
RIGHT_INDEX        = 270
BACK_RIGHT_INDEX   = 315
FINAL_BACK_INDEX   = 359

class WallFollowingStates(Enum):
    # nothing is detected, following wall, or moving past object
    STATE_MOVE_FORWARD = auto()
    # turning around a corner due to wall or object
    STATE_TURN_RIGHT    = auto()
    # reached a corner or moving around an object
    STATE_TURN_LEFT     = auto()
    # reached a dead end only option is to go backward
    STATE_MOVE_BACKWARD = auto()
    # attempted to go backward unsuccessfully, robot is stuck
    STATE_STUCK         = auto()

class WallFollower(Node):
    '''
    '''

    def __init__(self):
        '''
        The initializing function of the wall follower node.
        '''
        # Initialize the publisher via parent class
        super().__init__('wall_follower_node')

        # stores most up-to-date odometry and laser scan data
        self.scan_cleaned  = []
        self.pose          = ''
        self.prev_pose     = ''

        self.x_change_since_last_report = 0
        self.y_change_since_last_report = 0

        # stores the odometry data to be stored, this should be updated based on
        # whether we detect we are stuck or not.
        self.actual_x_pose = 0
        self.actual_y_pose = 0

        # variables used in to initiate and handle the error recovery state
        self.duplicate_count = 0
        self.recovery_cycles = 0

        # will be used to send commands to the robot
        self.cmd = Twist()

        # the state that the robot is currently in, this value will be
        # overwritten
        self.state          = WallFollowingStates.STATE_MOVE_FORWARD
        self.previous_state = self.state

        # initialize the publisher for velcity commands
        self.publisher_  = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # initialize the subscription to laser data
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # initialize the subscription to odometry data
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # amount of time between checks for updates
        timer_period = 0.5
        # the time that will cause the callback to execute
        self.timer   = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        '''
        Callback that executes upon new LaserScan data being received. Stores
        the new data in instance variables for use.
        Parameters:
        -----------
            msg1: The msg received via the LaserScan subscription.
        Returns:
        --------
            None
        '''
        # Erase stale laser range data
        self.scan_cleaned = []

        # Parse the laser data from the message
        scan = msg1.ranges

        # Loop through range data for cleaning (assumes 360 degrees)
        for reading in scan:
            # If the reading is infiinty set to maximum range
            if float('Inf') == reading:
                self.scan_cleaned.append(3.5)
            # If the reading is NaN assume range is zero
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            # Otherwise assume reading is valid
            else:
                self.scan_cleaned.append(reading)

        return

    def listener_callback2(self, msg2):
        '''
        Callback that executes upon new Odometry data being received. Stores
        the new data in instance varaible for use.

        Parameters:
        -----------
            msg2: The msg revieved via the Odometry subscription.
        Returns:
        --------
            None
        '''

        # Get the current position and orientation information
        position    = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation

        # Parse out the data from the vectors
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw)   = (orientation.x, orientation.y, orientation.z, orientation.w)

        if self.pose == '':
            self.prev_pose = position
        else:
            self.prev_pose = self.pose

        # Save the current position data for future use
        self.pose = position

        self.x_change_since_last_report += self.pose.x - self.prev_pose.x
        self.y_change_since_last_report += self.pose.y - self.prev_pose.y

        return

    def object_detected(self, min_foward_lidar, min_left_lidar, min_right_lidar, min_backward_lidar):
        '''
        Determines if an object is detected within the specified range or not.
        '''
        b_forward_object  = min_foward_lidar   < FORWARD_DETECT_RANGE
        b_left_object     = min_left_lidar     < LEFT_DETECT_RANGE
        b_right_object    = min_right_lidar    < RIGHT_DETECT_RANGE
        b_backward_object = min_backward_lidar < BACKWARD_DETECT_RANGE

        return (b_forward_object, b_left_object, b_right_object, b_backward_object)

    def get_state(self):
        '''
        Determines what state the robot is in based on the laser scan data.

        @note This functions assumes the lidar data stored in instance variable
              is up to date and accurate.

        Returns:
        --------
            WallFollowingStates:
                An enum variable representing which state that the robot is
                currently in.
        '''
        # Get the lidar data for the different regions
        left_lidar_data       = self.scan_cleaned[BACK_LEFT_INDEX:FRONT_LEFT_INDEX]
        right_lidar_data      = self.scan_cleaned[FRONT_RIGHT_INDEX:BACK_RIGHT_INDEX]
        front_lidar_data      = self.scan_cleaned[FRONT_LEFT_INDEX:FRONT_RIGHT_INDEX]
        back_left_lidar_data  = self.scan_cleaned[INITIAL_BACK_INDEX:BACK_LEFT_INDEX]
        back_right_lidar_data = self.scan_cleaned[BACK_RIGHT_INDEX:FINAL_BACK_INDEX]
        back_lidar_data       = back_left_lidar_data + back_right_lidar_data

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

    def timer_callback(self):
        '''
        Callback function that will execute once during the specified time
        period. This will collect the laser and odometry data before calling
        the current state's function.
        '''
        # if we have not yey received any laser data exit immediantely
        if (len(self.scan_cleaned) == 0) or (self.pose == ''):
            return

        # if the same position has not been detected 10 times, continue execution
        # as normal
        if self.duplicate_count < MAX_DUP_POSITIONS:

            # determine which state we are in
            self.previous_state = self.state
            self.state          = self.get_state()

            # if we are in the same state, don't report position change if so
            if self.previous_state == self.state:
                self.get_logger().info('Duplicate State Detected')
                self.duplicate_count += 1
                if self.duplicate_count < MAX_DUP_POSITIONS / 2:
                    self.actual_x_pose += self.x_change_since_last_report
                    self.actual_y_pose += self.y_change_since_last_report
                    self.x_change_since_last_report = 0
                    self.y_change_since_last_report = 0
                    # write the coordinates to a file
                    f = open('coordinates.txt', 'a+')
                    f.write(f'({self.actual_x_pose},{self.actual_y_pose})\n')
                    f.close()

            # if we're are not in the same state, assume we are moving correctly
            # and report the position change.
            else:
                self.get_logger().info('New State Detected')
                self.duplicate_count = 0
                self.actual_x_pose += self.x_change_since_last_report
                self.actual_y_pose += self.y_change_since_last_report
                self.x_change_since_last_report = 0
                self.y_change_since_last_report = 0
                # write the coordinates to a file
                f = open('coordinates.txt', 'a+')
                f.write(f'({self.actual_x_pose},{self.actual_y_pose})\n')
                f.close()

            if WallFollowingStates.STATE_MOVE_FORWARD == self.state:
                self.get_logger().info('Entering Move Forward State')
                self.cmd.linear.x  = NORMAL_FORWARD_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_FORWARD_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_MOVE_BACKWARD == self.state:
                self.get_logger().info('Entering Move Backward State')
                self.cmd.linear.x  = NORMAL_BACKWARD_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_BACKWARD_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_TURN_RIGHT == self.state:
                self.get_logger().info('Entering Turn Right State')
                self.cmd.linear.x  = NORMAL_TURNING_RIGHT_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_TURNING_RIGHT_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_TURN_LEFT == self.state:
                self.get_logger().info('Entering Turn Left State')
                self.cmd.linear.x  = NORMAL_TURNING_LEFT_LINEAR_SPEED
                self.cmd.angular.z = NORMAL_TURNING_LEFT_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            elif WallFollowingStates.STATE_STUCK == self.state:
                self.get_logger().info('Entering Stuck State')
                self.cmd.linear.x  = STUCK_LINEAR_SPEED
                self.cmd.angular.z = STUCK_ANGULAR_SPEED
                self.publisher_.publish(self.cmd)

            else:
                self.get_logger().info('Invalid State Detected!')

        else:

            # attempting to recover, reset change values
            if self.recovery_cycles == 0:
                self.x_change_since_last_report = 0
                self.y_change_since_last_report = 0

            self.actual_x_pose += self.x_change_since_last_report
            self.actual_y_pose += self.y_change_since_last_report
            self.x_change_since_last_report = 0
            self.y_change_since_last_report = 0

            # write the coordinates to a file
            f = open('coordinates.txt', 'a+')
            f.write(f'({self.actual_x_pose},{self.actual_y_pose})\n')
            f.close()

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
                self.duplicate_count         = 0
                self.recovery_cycles         = 0

            self.recovery_cycles += 1

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
