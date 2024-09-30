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

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class MovementTrials(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.subscriber1 = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.listener_callback1,
        #     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)


    # def listener_callback1(self, msg1):
    #     #self.get_logger().info('scan: "%s"' % msg1.ranges)
    #     scan = msg1.ranges
    #     self.scan_cleaned = []
    #    
    #     #self.get_logger().info('scan: "%s"' % scan)
    #     # Assume 360 range measurements
    #     for reading in scan:
    #         if reading == float('Inf'):
    #             self.scan_cleaned.append(3.5)
    #         elif math.isnan(reading):
    #             self.scan_cleaned.append(0.0)
    #         else:
    #             self.scan_cleaned.append(reading)


    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        self.pose_saved=position
        
        return None
        
    def timer_callback(self):
        self.cmd.linear.x = 0.3
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
