from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to YOLO model weights (.pt).'
    )

    return LaunchDescription([
        model_path_arg,
        Node(
            package='cube_detection',
            executable='cube_detector_node',
            name='cube_detector_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
            }]
        ),
        Node(
            package='cube_detection',
            executable='cube_chase_controller',
            name='cube_chase_controller',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
            }]
        )
    ])