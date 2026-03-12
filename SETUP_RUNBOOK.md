# MKS Rover Setup Runbook

This runbook covers laptop-to-Jetson connection, ROS environment setup, and orchestration bring-up.

## 1) Network Prep (Laptop)

1. Plug laptop Ethernet into the base station radio.
2. Disable Wi-Fi.
3. Verify rover reachability (use the Jetson's IP):

```bash
ping 192.168.1.161
```

## 2) SSH Into Jetson

```bash
ssh rover@192.168.1.161
```

Then start tmux:

```bash
tmux
```

## 3) Jetson Sanity Checks

```bash
ip a
cd ~
find ~ -maxdepth 3 -type f -name "setup.bash" 2>/dev/null | head -n 20
```

Source ROS + workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
```

Confirm package visibility:

```bash
ros2 pkg list | grep mks_control
```

Expected: `mks_control` appears.

## 4) Start Jetson Nodes (Two tmux Panes)

Pane 1:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash

ros2 launch mks_control teleop_drive.launch.py
```

Pane 2:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 node list
echo $ROS_DOMAIN_ID
```

## 5) Match ROS Domain (Laptop <-> Jetson)

On Jetson, note:

```bash
echo $ROS_DOMAIN_ID
```

On laptop, match it:

```bash
export ROS_DOMAIN_ID=<same_as_jetson>
```

Example:

```bash
export ROS_DOMAIN_ID=0
```

## 6) Install Nav2 Prerequisites (One-Time)

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-nav2-bringup ros-humble-slam-toolbox
```

## 7) Run Orchestration From Laptop

Source your workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/mks_ws-main/install/setup.bash
```

Start the RealSense camera (RGB + depth + IMU):

```bash
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  enable_gyro:=true \
  enable_accel:=true
```

In a new terminal, start RGBD visual odometry (provides `/odom` + `odom → base_link` TF using depth camera directly):

```bash
source /opt/ros/humble/setup.bash
source ~/mks_ws-main/install/setup.bash
ros2 run rtabmap_odom rgbd_odometry \
  --ros-args \
  -r rgb/image:=/camera/camera/color/image_raw \
  -r rgb/camera_info:=/camera/camera/color/camera_info \
  -r depth/image:=/camera/camera/depth/image_rect_raw \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=true \
  -p wait_for_transform:=0.2
```

In a new terminal, start the frontier explorer:

```bash
source /opt/ros/humble/setup.bash
source ~/mks_ws-main/install/setup.bash
ros2 run mks_orchestration frontier_service_node
```

In another terminal, start the orchestrator:

```bash
source /opt/ros/humble/setup.bash
source ~/mks_ws-main/install/setup.bash
ros2 run mks_orchestration high_orchestration_node
```

**Alternative:** Launch frontier explorer + orchestrator together (you still need to start depth-to-scan and rgbd odometry separately above):

```bash
ros2 launch mks_orchestration autonomy_orchestration.launch.py
```

## 8) Verify ROS Graph and Command Flow

On laptop:

```bash
source /opt/ros/humble/setup.bash
source ~/mks_ws-main/install/setup.bash

ros2 node list
ros2 topic echo /cmd_vel
```

On Jetson (optional cross-check):

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

## 9) If SSH Drops

Reconnect and reattach tmux:

```bash
ssh rover@172.19.103.197
tmux attach
```

If multiple sessions exist:

```bash
tmux ls
tmux attach -t <session_name>
```

## Quick Troubleshooting

- `ros2` command not found:
  - Re-run `source /opt/ros/humble/setup.bash`.
- Package not found:
  - Re-run workspace source (`~/rover_ws/install/setup.bash` on Jetson, `~/mks_ws-main/install/setup.bash` on laptop).
- No cross-machine topics/nodes:
  - Confirm both sides have same `ROS_DOMAIN_ID`.
  - Re-check Ethernet path and `ping 192.168.1.161`.
