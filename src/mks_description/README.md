# Instructions For Visualizing Robot


1) Build the workspace/package with `colcon build`
2) Launch the robot_state_publisher launch file with `ros2 launch mks_description rsp.launch.py`
3) Launch joint_state_publisher_gui with `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
4) Launch RViz with `rviz2`

## Configuring RVIZ

1) Set base link to base link (dropdown in top left)
2) Add `RobotModel` display (add button in bottom left)
3) For point #2, set description source to `Topic` and set topic to `/robot_description`

## Useful Debugging Commands:

- `ros2 run xacro xacro /home/mksneo/mks_ws/src/mks_description/urdf/eve.urdf.xacro`

OR

- `ros2 run xacro xacro /home/mksneo/mks_ws/src/mks_description/urdf/eve.urdf.xacro > /home/mksneo/mks_ws/src/mks_description/urdf/eve.urdf`

To SDF
- `gz sdf -p /home/mksneo/mks_ws/src/mks_description/urdf/eve.urdf > /home/mksneo/mks_ws/src/mks_description/sdf/eve.sdf`

## STL Files

Should go in `/mks_description/meshes` 

You can access them here:
- https://drive.google.com/drive/folders/1dvsBxLHqj5qetu0XFq9vR1L56kMXGcLv?usp=sharing

## Ignition:
- empty world: `ign gazebo -v 4 -r empty.sdf`

## Source:
- https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html


```bash
ros2 launch gz_ros2_control_demos cart_example_position.launch.py
ros2 launch gz_ros2_control_demos cart_example_velocity.launch.py
ros2 launch gz_ros2_control_demos cart_example_effort.launch.py
```

```bash
ros2 run gz_ros2_control_demos example_position
ros2 run gz_ros2_control_demos example_velocity
ros2 run gz_ros2_control_demos example_effort
```

https://github.com/ros-controls/gz_ros2_control/tree/rolling/gz_ros2_control_demos