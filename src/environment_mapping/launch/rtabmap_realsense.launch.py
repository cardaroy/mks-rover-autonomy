from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/camera/imu/data'),

        Node(
            package='rtabmap',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }],
            remappings=[('/camera/color/image_raw', '/camera/color/image_raw'),
                        ('/camera/depth/image_raw', '/camera/depth/image_rect_raw'),
                        ('/camera/imu/data', '/camera/imu/data')]
        )
    ])
