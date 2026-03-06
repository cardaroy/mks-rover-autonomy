from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_topic_arg = DeclareLaunchArgument('map_topic', default_value='/map')
    nav_action_arg = DeclareLaunchArgument('navigate_action', default_value='/navigate_to_pose')
    target_cube_count_arg = DeclareLaunchArgument('target_cube_count', default_value='4')

    frontier_node = Node(
        package='mks_orchestration',
        executable='frontier_service_node',
        name='frontier_service_node',
        output='screen',
        parameters=[{
            'map_topic': LaunchConfiguration('map_topic'),
            'navigate_action': LaunchConfiguration('navigate_action'),
        }],
    )

    orchestrator_node = Node(
        package='mks_orchestration',
        executable='high_orchestration_node',
        name='high_orchestration_node',
        output='screen',
        parameters=[{
            'target_cube_count': LaunchConfiguration('target_cube_count'),
        }],
    )

    return LaunchDescription([
        map_topic_arg,
        nav_action_arg,
        target_cube_count_arg,
        frontier_node,
        orchestrator_node,
    ])
