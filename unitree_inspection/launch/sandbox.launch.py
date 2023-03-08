from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('unitree_nav'),
        #             'launch',
        #             'control.launch.py'
        #         ])
        #     ),
        # ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='unitree_inspection',
            executable='sandbox',
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