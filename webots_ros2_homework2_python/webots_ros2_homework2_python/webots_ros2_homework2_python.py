import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
from enum import Enum, auto

class TrialTypes(Enum):
    DISTANCE_TRIAL = auto()
    TURNING_TRIAL  = auto()

# 75 mm/second
TRIAL_ONE_LINEAR_SPEED    = 0.075
# 150 mm/second
TRIAL_TWO_LINEAR_SPEED    = 0.15

# 30 degrees/second (in radians)
TRIAL_THREE_TURNING_SPEED = 0.523599
# 120 degrees/second (in radians)
TRIAL_FOUR_TURNING_SPEED  = 2.0944

# Distance Trials (meters)
DISTANCE_ONE    = 1.0
DISTANCE_TWO    = 5.0

# Degree Trials (radians)
DEGREE_ONE      = 0.174533 # 10 degrees
DEGREE_TWO      = 3.14159  # 180 degrees
DEGREE_THREE    = 6.28319  # 360 degrees

# NOTE: Set these values to execute a test
TEST_TYPE          = TrialTypes.TURNING_TRIAL
DISTANCE_TO_TRAVEL = 0.0
LINEAR_SPEED       = 0.0
DEGREES_TO_TURN    = DEGREE_THREE
TURNING_SPEED      = TRIAL_THREE_TURNING_SPEED

class MovementTrials(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('walk_node')

        self.distance_traveled = 0
        self.radians_turned    = 0
        self.test_completed    = False

        self.publisher_  = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscriber1 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Stores the most recent position and orientation data
        self.pose_saved   = ''
        self.orient_saved = ''

        self.cmd = Twist()

        timer_period = 0.075
        self.timer   = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        '''
        Callback that is executed when new odomoetry data is available.

        Parameters:
        -----------
            msg1: The new odometry position data.
        '''
        # Get the current position as reported by the turtlebot
        position    = msg1.pose.pose.position
        # Get the current orientation as reported by the turtlebot
        orientation = msg1.pose.pose.orientation

        # Store the current position and orientation data
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw)   = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Compute distance traveled and degrees turned
        if self.pose_saved != '':
            partial_distance_traveled    = (posx - self.pose_saved.x)
            self.distance_traveled      += abs(partial_distance_traveled)
            partial_radians_turned       = (qz - self.orient_saved.z)
            self.radians_turned         += abs(partial_radians_turned)

            if partial_distance_traveled != 0.0 and TEST_TYPE == TrialTypes.DISTANCE_TRIAL:
                self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
                self.get_logger().info(f'distance since last report: {partial_distance_traveled}')
                self.get_logger().info(f'distance traveled: {self.distance_traveled}')
            elif partial_radians_turned != 0.0 and TEST_TYPE == TrialTypes.TURNING_TRIAL:
                self.get_logger().info('self orientation: {} {} {}'.format(qx, qy, qz))
                self.get_logger().info(f'radians turned since last report: {partial_radians_turned}')
                self.get_logger().info(f'radians turned: {self.radians_turned}')

        # similarly for twist message if you need
        self.pose_saved   = position
        self.orient_saved = orientation

        return None

    def timer_callback(self):
        '''
        Callback function that is called once every timer_period seconds.
        '''
        if not self.test_completed:
            self.cmd.linear.x  = LINEAR_SPEED
            self.cmd.angular.z = TURNING_SPEED

            if TrialTypes.DISTANCE_TRIAL == TEST_TYPE:
                if self.distance_traveled >= DISTANCE_TO_TRAVEL:
                    self.get_logger().info(f'Stopping Test')
                    self.test_completed = True
            elif TrialTypes.TURNING_TRIAL == TEST_TYPE:
                if self.radians_turned >= DEGREES_TO_TURN:
                    self.get_logger().info('Stopping Test')
                    self.test_completed = True

        if self.test_completed:
            self.cmd.linear.x  = 0.0
            self.cmd.angular.z = 0.0

        self.publisher_.publish(self.cmd)

        return None

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    walk_node = MovementTrials()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(walk_node)
    # Explicity destroy the node
    walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
