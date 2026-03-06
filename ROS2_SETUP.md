## ROS2 Workspace Setup (mks_ws-main)

This guide sets up and runs the workspace packages, including:
- `mks_description` (Gazebo/Ignition + ros2_control)
- `mks_control` (joystick -> `/cmd_vel` -> wheel commands)
- `cube_detection` (YOLO cube detector + chase)
- `mks_orchestration` (high-level BT mission node)

---

## 1) Prerequisites

Use Ubuntu (native or WSL2) with ROS2 installed (recommended: Humble/Jazzy, matching your stack).

Required tools:
- `colcon`
- `vcstool` (optional)
- `rosdep`
- Gazebo/ROS-GZ packages used by your distro

Example package install (Ubuntu):

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Initialize rosdep (one-time):

```bash
sudo rosdep init
rosdep update
```

---

## 2) Build the Workspace

From workspace root (`mks_ws-main/mks_ws-main`):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you are on Windows PowerShell with ROS2 for Windows, use the equivalent `.bat`/PowerShell sourcing flow.

---

## 3) Model Path Setup for YOLO

The detector now uses a launch arg or env var instead of hardcoded paths.

Set model path with either:

### Option A: env var
```bash
export CUBE_MODEL_PATH=/absolute/path/to/best.pt
```

### Option B: launch argument
Pass `model_path:=...` when launching `cube_detection`.

---

## 4) Run Core Simulation + Controllers

Terminal A:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch mks_description ignition.launch.py
```

This spawns robot + controller manager + `wheel_velocity_controller`.

---

## 5) Run Teleop-to-Drive Pipeline

Terminal B:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch mks_control teleop_drive.launch.py
```

Optional tuning:
```bash
ros2 launch mks_control teleop_drive.launch.py \
  joy_max_linear_speed:=1.0 joy_max_angular_speed:=1.0 \
  drive_max_linear_speed:=2000.0 drive_max_angular_speed:=2000.0
```

---

## 6) Run Cube Detection + Chase

Terminal C:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch cube_detection cube_detection.launch.py model_path:=$CUBE_MODEL_PATH
```

---

## 7) Run Orchestration Node

Terminal D:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run mks_orchestration high_orchestration_node
```

---

## 8) Quick Verification Commands

```bash
ros2 pkg list | grep -E "mks_orchestration|mks_control|mks_description|cube_detection|environment_mapping"
ros2 topic list
ros2 control list_controllers
ros2 topic echo /orchestrator/state
```

Expected controller:
- `wheel_velocity_controller` should be active after ignition launch.

---

## 9) Common Issues

1. `colcon: command not found`
- Install `python3-colcon-common-extensions`.

2. ROS Python imports unresolved in editor
- VS Code interpreter is not your ROS Python env; runtime may still be fine when sourced correctly.

3. `Model not found` from cube detector
- Set `CUBE_MODEL_PATH` or pass launch arg `model_path:=...`.

4. Package not found after adding files
- Rebuild and re-source:
```bash
colcon build --symlink-install
source install/setup.bash
```
