# MKS Rover Workspace - AI Coding Guidelines

## Project Overview

This is a **ROS 2 (Humble) robotics workspace** for the MKS rover—a quad-legged robot with computer vision, SLAM, and motor control capabilities. The workspace is structured as a colcon build system with Python and C++ packages.

### Key Hardware
- **Robot**: EVE rover with 4 motorized legs, IMU, and on-board compute
- **Sensors**: Intel RealSense D435i camera (RGB-D + IMU)
- **GPU**: RTX 5070 (Gazebo simulation support)
- **OS**: Ubuntu with ROS 2 Humble

---

## Architecture Overview

### Core Packages

**mks_description** (C++ - CMake)
- Robot URDF definition in `urdf/eve.urdf.xacro`
- Uses xacro for parameterized robot model with mesh paths
- Includes simplified meshes for collision geometry (`simplified_meshes/`)
- Launch files: `rsp.launch.py` (robot_state_publisher), `ignition.launch.py` (Gazebo)
- **Key pattern**: Mesh paths are currently hardcoded to absolute paths (e.g., `/home/mksneo/mks_ws/...`) — see [summary_failed_approaches.md](summary_failed_approaches.md) for migration challenges

**mks_control** (C++ - ROS 2 Control)
- `cmd_vel_to_drive.cpp`: Converts `/cmd_vel` (Twist) → skid-steer motor commands
- Implements **skid-steer kinematics**: `left_speed = lin - ang`, `right_speed = lin + ang`
- Publishes to `front_left/commands/motor/speed`, etc. (4 wheels)

**cube_detection** (Python - ROS 2)
- YOLO object detection node: detects cubes from camera feed
- Input: RealSense color + depth from `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`
- Output: `PoseArray` of detected cube positions (3D from depth)
- Uses **cv_bridge** for image conversion, **ultralytics YOLO** for inference
- Launch: `cube_detection.launch.py` starts detector + chase controller

**environment_mapping** (Python - ROS 2 + RTABmap)
- Accumulates point clouds from RealSense depth sensor
- `pointcloud_accumulator.py`: Subscribes to `/camera/depth/color/points` (IMU-enhanced)
- RTABmap integration for SLAM (launch: `rtabmap_realsense.launch.py`)
- Prerequisites: `realsense2_camera`, `rtabmap` ROS packages

**py_talker_listener** (Python - Example)
- Basic ROS 2 pub/sub example (can ignore for production)

**cpp_number_sum** (C++ - Example)
- Simple C++ ROS example (can ignore for production)

---

## Critical Workflows

### Building & Running
```bash
# Build entire workspace
colcon build

# Build specific package
colcon build --packages-select cube_detection

# Source install (required after build)
source install/setup.bash
```

### Visualizing Robot
1. `ros2 launch mks_description rsp.launch.py` — publishes `/robot_description`
2. `ros2 run joint_state_publisher_gui joint_state_publisher_gui` — interactive joint control (optional)
3. `rviz2` → Add RobotModel display → Topic: `/robot_description`

### Running SLAM
- Prerequisites: `sudo apt install ros-humble-realsense2-camera ros-humble-rtabmap-ros`
- Connect RealSense D435i via USB 3.0
- Verify with `lsusb | grep Intel`
- Launch: `ros2 launch environment_mapping rtabmap_realsense.launch.py`
- View in RViz: add PointCloud2 display on `/accumulated_pointcloud`

### Debugging URDF/Xacro
```bash
# Expand xacro to URDF
ros2 run xacro xacro /path/to/eve.urdf.xacro > /tmp/eve.urdf

# Convert URDF to SDF (for Gazebo)
gz sdf -p /tmp/eve.urdf > /tmp/eve.sdf

# Check for mesh errors in Gazebo
ign gazebo empty.sdf  # or spawned robot in Gazebo
```

### Gazebo Simulation
- GPU rendering requires RTX driver: `nvidia-driver-590-open`
- Set environment for GL3: `export OGRE_RHI=GL3`
- Launch simulation: `ros2 launch mks_description spawn.launch.py` or `ignition.launch.py`

---

## Project-Specific Conventions

### ROS Topics & Message Types
| Component | Publishes | Subscribes | Message Type |
|-----------|-----------|-----------|--------------|
| cube_detector | `/detected_cubes` | `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw` | PoseArray |
| cmd_vel_to_drive | `front_left/commands/motor/speed`, etc. | `/cmd_vel` | Float64 |
| pointcloud_accumulator | `/accumulated_pointcloud` | `/camera/depth/color/points`, `/camera/imu` | PointCloud2, Imu |

### YOLO Model Loading
- Model path is a **ROS parameter** (see `cube_detector_node.py` line 30)
- Default: `/home/yao/models/best.pt`
- Confidence threshold: 0.6 (configurable parameter)
- Depth filtering: 0.15m (min) to 5.0m (max)

### Motor Control
- **Not direct GPIO drivers**—expects intermediate hardware abstraction layer
- Subscribes to abstract "motor speed" commands (0–2000 range, configured by max_linear/angular_speed params)
- Skid-steer kinematic model assumes 4-wheel rover with tank-like turning

### Mesh Loading Issues ⚠️
- **Hardcoded absolute paths** in `eve.urdf.xacro` (line 5): problematic for portability
- Migration to `package://` URLs failed in past attempts (see summary_failed_approaches.md)
- **Workaround**: Set `GZ_SIM_RESOURCE_PATH` in bashrc:
  ```bash
  export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/mks_description/share/mks_description
  ```

---

## Integration Points & Dependencies

### RealSense Camera
- ROS package: `realsense2_camera` publishes color/depth/IMU on fixed topics
- Topics: `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`, `/camera/imu`
- Depth units: 16-bit unsigned, typically in mm (configurable in `cube_detector_node.py`)

### RTABMAP (SLAM)
- External dependency: `ros-humble-rtabmap-ros`
- Node consumes RGB-D + IMU → outputs `/rtabmap/cloud_map` (accumulated point cloud)
- **Note**: `pointcloud_accumulator.py` duplicates accumulation; consider consolidating with RTABmap

### ROS 2 Control Framework
- Not explicitly used yet—motor commands are direct publisher/subscriber
- Future: Consider `ros2_control` with controller managers for better abstraction

---

## File Organization Patterns

### Launch Files
- All in `package_name/launch/*.launch.py`
- Use xacro for URDF processing (see `rsp.launch.py`)
- Declare launch args with `LaunchConfiguration` for parameterization

### Python Nodes
- Entry point in `setup.py` maps executable name to module function
- All nodes inherit from `rclpy.node.Node`
- Use `self.declare_parameter()` + `self.get_parameter()` for ROS params

### URDF/Xacro
- Links: define visual (mesh), collision (simplified mesh), inertial (mass/moment)
- Origins in visual/collision account for mesh offsets (rpy in radians)
- **Macro example** (unused but available): `<xacro:arg name="mesh_path" ... />`

---

## Key Files to Know

- [eve.urdf.xacro](src/mks_description/urdf/eve.urdf.xacro) — robot structure
- [cube_detector_node.py](src/cube_detection/cube_detection/cube_detector_node.py) — main vision pipeline
- [cmd_vel_to_drive.cpp](src/mks_control/src/cmd_vel_to_drive.cpp) — motor control
- [rsp.launch.py](src/mks_description/launch/rsp.launch.py) — robot state pub pattern
- [pointcloud_accumulator.py](src/environment_mapping/environment_mapping/pointcloud_accumulator.py) — SLAM data processing
- [driver_info.md](driver_info.md) — GPU driver setup for RTX 5070
- [summary_failed_approaches.md](summary_failed_approaches.md) — known issues & past attempts

---

## Testing & Debugging Tips

1. **Topic monitoring**: `ros2 topic list`, `ros2 topic echo /camera/camera/color/image_raw`
2. **Node status**: `ros2 node list`, `ros2 node info /cube_detector_node`
3. **Parameter inspection**: `ros2 param list /cube_detector_node`
4. **Bag recording** (for offline testing):
   ```bash
   ros2 bag record /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw
   ros2 bag play rosbag_folder
   ```
5. **YOLO inference debug**: Add print statements in `cube_detector_node.py` around line 120+ for detection results

---

## Common Pitfalls

- **Mesh paths**: Absolute paths break across machines—use environment variables or launch parameter substitution
- **RealSense not detected**: Check USB port is 3.0, run `lsusb`, install realsense SDK if needed
- **RTABmap node crashes**: Ensure RGB-D topics are remapped correctly in launch file
- **Motor commands not received**: Check `/cmd_vel` topic exists and motor publishers are subscribed
- **URDF parsing errors**: Run xacro expansion and check for undefined variables

