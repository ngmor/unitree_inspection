from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='unitree_inspection',
            executable='inspection',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('unitree_inspection'),
                    'config',
                    'environment_params.yaml'
                ]),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('unitree_ocr'),
                    'launch',
                    'text_detection_subscriber.launch.py'
                ])
            ),
        ),
    ])