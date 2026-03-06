from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    slam_arg = DeclareLaunchArgument('slam', default_value='true')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=[FindPackageShare('mks_navigation'), '/config/nav2_params.yaml'],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'),
            '/launch/bringup_launch.py',
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'slam': LaunchConfiguration('slam'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([use_sim_time_arg, autostart_arg, slam_arg, params_file_arg, nav2_bringup])
