from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes

def generate_launch_description():

    # controller node to be launched
    turtlebot_controller_node = Node(
        name='webots_ros2_homework4_python',
        package='webots_ros2_homework4_python',
        executable='webots_ros2_homework4_python',
        output='screen'
    )

    # camera node to be launched
    camera_node = Node(
        name='v4l2_camera_node',
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen'
    )

    # april tag node to be launched
    april_tag_node = Node(
        name='apriltag',
        package='apriltag_ros',
        executable='apriltag_ros',
        parameters=['/opt/ros/humble/share/apriltag_ros/cfg/tags_36h11.yaml'],
        remappings=[('/image_rect', '/image_raw'), ('/camera_info', '/camera_info')],
        output='screen'
    )

    # image view node to be launched
    image_view_node = Node(
        package='v4l2_camera',
        executable='rqt_image_view',
        output='screen'
    )

    return LaunchDescription([
        turtlebot_controller_node,
        camera_node,
        april_tag_node,
        image_view_node
    ])
