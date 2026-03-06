from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_linear_arg = DeclareLaunchArgument(
        'joy_max_linear_speed',
        default_value='1.0',
        description='Max linear speed scaling for joy_to_cmd_vel',
    )
    joy_angular_arg = DeclareLaunchArgument(
        'joy_max_angular_speed',
        default_value='1.0',
        description='Max angular speed scaling for joy_to_cmd_vel',
    )
    drive_linear_arg = DeclareLaunchArgument(
        'drive_max_linear_speed',
        default_value='2000.0',
        description='Linear cmd_vel to wheel command scale in cmd_vel_to_drive',
    )
    drive_angular_arg = DeclareLaunchArgument(
        'drive_max_angular_speed',
        default_value='2000.0',
        description='Angular cmd_vel to wheel command scale in cmd_vel_to_drive',
    )

    joy_node = Node(
        package='mks_control',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('joy_max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('joy_max_angular_speed'),
        }],
    )

    drive_node = Node(
        package='mks_control',
        executable='cmd_vel_to_drive',
        name='cmd_vel_to_drive',
        output='screen',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('drive_max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('drive_max_angular_speed'),
        }],
    )

    return LaunchDescription([
        joy_linear_arg,
        joy_angular_arg,
        drive_linear_arg,
        drive_angular_arg,
        joy_node,
        drive_node,
    ])
