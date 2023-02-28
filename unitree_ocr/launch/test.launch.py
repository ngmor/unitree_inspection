# For testing on a device with ROS's usb_cam package

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('usb_cam'),
                    'config',
                    'params.yaml',
                ]),
            ],
            remappings=[
                ('/image_raw', '/head/front/cam/left/image_rect')
            ],
        ),
        Node(
            package='usb_cam',
            executable='show_image.py',
            output='screen',
            remappings=[
                ('/image_raw', '/head/front/cam/left/image_rect')
            ],
        )
    ])