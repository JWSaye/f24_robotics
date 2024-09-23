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
import csv



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
DESIRED_WALL_DISTANCE = 0.5  # Desired distance from the wall (in meters)
DOORWAY_THRESHOLD = 0.5  # Change in distance to detect doorway

class WallFollow(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('wall_follow_node')
        self.scan_cleaned = []
        self.prev_right_lidar_min = None  # To store the previous right LIDAR min value
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
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
        
        # Open CSV file to log position data
        self.csv_file = open('Homework1/Zone1_Run5_turtlebot_positions.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'z'])  # Write the header row


    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
        
        # Log the received LIDAR data
        # self.get_logger().info(f'Received LIDAR data: {scan[:10]}')  # Log the first 10 readings
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)
        
        # self.get_logger().info(f'Cleaned LIDAR data: {self.scan_cleaned[:10]}')  # Log the cleaned data


    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        # self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        
        # Log position to CSV
        self.csv_writer.writerow([posx, posy, posz])
        self.pose_saved=position
            
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None
        
    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            return

        # Get the right and front LIDAR minimum values
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # Check if there's an increase in the right LIDAR min (indicating an open door)
        if self.prev_right_lidar_min is not None:
            if right_lidar_min - self.prev_right_lidar_min > DOORWAY_THRESHOLD:
                self.get_logger().info("Doorway detected! Moving straight through.")
                self.cmd.linear.x = 0.1  # Move forward slower to go through the door
                self.cmd.angular.z = -0.5  # Turn to doorway
                self.publisher_.publish(self.cmd)
                return  # Exit early to keep moving straight
            
        # If no doorway detected, follow the wall as usual
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.get_logger().info("Obstacle detected! Stopping.")
            # Stop if an obstacle is in front
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5  # Turn left to avoid the obstacle
        elif right_lidar_min < DESIRED_WALL_DISTANCE - LIDAR_ERROR:
            self.get_logger().info("Too close to the wall! Turning left.")
            # Too close to the wall, turn left
            self.cmd.linear.x = 0.2  # Move forward slowly
            self.cmd.angular.z = 0.3  # Turn left slightly
        elif right_lidar_min > DESIRED_WALL_DISTANCE + LIDAR_ERROR:
            self.get_logger().info("Too far from the wall! Turning right.")
            # Too far from the wall, turn right
            self.cmd.linear.x = 0.2  # Move forward slowly
            self.cmd.angular.z = -0.3  # Turn right slightly
        else:
            self.get_logger().info("Correct distance from the wall! Moving straight.")
            # Correct distance from the wall, move straight
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0

        # Publish the velocity command
        self.publisher_.publish(self.cmd)

        # Update the previous right LIDAR minimum value for the next cycle
        self.prev_right_lidar_min = right_lidar_min

 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_follow_node = WallFollow()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_follow_node)
    # Explicity destroy the node
    wall_follow_node.destroy_node()
    # Close the CSV file
    wall_follow_node.csv_file.close()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
