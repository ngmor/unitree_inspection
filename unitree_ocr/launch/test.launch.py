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
            package='unitree_ocr',
            executable='text_detection_subscriber',
            output='screen',
            parameters=[{
                'detection.model_path':
                    PathJoinSubstitution([
                        FindPackageShare('unitree_ocr'),
                        'models',
                        'frozen_east_text_detection.pb',
                    ]),
                'recognition.model_path':
                    PathJoinSubstitution([
                        FindPackageShare('unitree_ocr'),
                        'models',
                        'crnn_cs.onnx',
                    ]),
                'recognition.vocabulary_path':
                    PathJoinSubstitution([
                        FindPackageShare('unitree_ocr'),
                        'models',
                        'alphabet_94.txt',
                    ]),
            }],
            remappings=[
                ('/image', '/head/front/cam/left/image_rect')
            ],
        )
    ])