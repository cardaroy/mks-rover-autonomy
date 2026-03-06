import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
import xacro


def generate_launch_description():

    mesh_path_for_gazebo = 'package://mks_description/meshes'
    spawn_height = 10

    pkg_name = 'mks_description'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'eve.urdf.xacro')

    # xacro -> urdf string
    robot_description_raw = xacro.process_file(xacro_file, mappings={'mesh_path': mesh_path_for_gazebo}).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    node_spawn_ignition = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'eve',
            '-string', robot_description_raw,
            '-z', str(spawn_height)
        ]
    )

    # Launching ignition
    world_path = os.path.join(
        get_package_share_directory('mks_description'),
        'worlds',
        'mks_world.sdf'
    )

    ign_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    launch_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ign_launch_file),
        launch_arguments={'gz_args': world_path}.items()
    )

    return LaunchDescription([
        launch_ignition,
        node_robot_state_publisher,
        node_spawn_ignition
    ])
