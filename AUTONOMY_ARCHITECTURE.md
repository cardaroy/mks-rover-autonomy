# Autonomy Architecture (MKS Rover)

This document explains how autonomy works in this workspace and where each part lives.

## Scope

There are two autonomy layers:

1. Reactive visual chasing (`cube_detection`): detect cubes and publish immediate `/cmd_vel`.
2. Mission orchestration (`mks_orchestration`): run frontier exploration until enough cubes are confirmed, then stop safely.

## Diagram 1: Full System Dataflow

```mermaid
flowchart LR
  subgraph Sensors
    RGB[RealSense RGB]
    DEPTH[RealSense Depth]
    CAMINFO[Camera Info]
    MAP[/map OccupancyGrid]
  end

  subgraph Perception
    DET[cube_detector_node\n(cube_detection)]
  end

  subgraph ReactiveControl
    CHASE[cube_chase_controller\n(cube_detection)]
  end

  subgraph MissionControl
    ORCH[high_orchestration_node\n(mks_orchestration)]
    FRONTIER[frontier_service_node\n(mks_orchestration)]
  end

  subgraph Navigation
    NAV2[NavigateToPose action server\n(nav2)]
  end

  subgraph Drive
    DRIVE[cmd_vel_to_drive\n(mks_control)]
    WHEEL[/wheel_velocity_controller/commands]
  end

  RGB --> DET
  DEPTH --> DET
  CAMINFO --> DET

  DET -->|/cube_poses| CHASE
  DET -->|/cube_poses| ORCH
  DET -->|/cube_detections_json, /cube_markers| RVIZ[RViz/Debug]

  ORCH -->|Trigger /frontier/start| FRONTIER
  ORCH -->|Trigger /frontier/stop| FRONTIER
  ORCH -->|/orchestrator/state| STATE[/orchestrator/state]
  ORCH -->|/orchestrator/cubes_json| CUBES[/orchestrator/cubes_json]

  MAP --> FRONTIER
  FRONTIER -->|NavigateToPose goal| NAV2

  CHASE -->|/cmd_vel| DRIVE
  NAV2 -->|/cmd_vel| DRIVE
  ORCH -->|safe stop /cmd_vel=0| DRIVE

  DRIVE --> WHEEL
```

## Diagram 2: Mission Behavior Tree

```mermaid
flowchart TD
  TICK[Tick at 10 Hz] --> SEQ[Sequence]

  SEQ --> PAR[ParallelAll]
  PAR --> A[EnsureFrontierExplorationRunning]
  PAR --> B[WaitForTargetCubeCount]

  A --> A1{Frontier already started?}
  A1 -- Yes --> A2[Success]
  A1 -- No --> A3[Call /frontier/start]\nA3 --> A4[Running]

  B --> B1{Confirmed cubes >= target?}
  B1 -- No --> B2[Running]
  B1 -- Yes --> B3[Success]

  PAR -->|both success| STOP[StopFrontierExploration]
  STOP --> S1{Frontier stopped?}
  S1 -- No --> S2[Call /frontier/stop]\nS2 --> S3[Running]
  S1 -- Yes --> SAFE[SafeStop]

  SAFE --> Z[Publish /cmd_vel = 0]
  Z --> DONE[Mission complete]
```

## Diagram 3: Motion Command Pipeline

```mermaid
flowchart LR
  CMD[/cmd_vel Twist]
  CVT[cmd_vel_to_drive]
  L[Left speed = lin - ang]
  R[Right speed = lin + ang]
  OUT[/wheel_velocity_controller/commands\n[Left, Right, Left, Right]]

  CMD --> CVT
  CVT --> L
  CVT --> R
  L --> OUT
  R --> OUT
```

## Execution Walkthrough

1. `cube_detector_node` reads RGB + depth + intrinsics, runs YOLO, and publishes 3D cube poses on `/cube_poses`.
2. `cube_chase_controller` uses nearest cube pose (`min Z`) for visual servoing and publishes `/cmd_vel`.
3. `high_orchestration_node` also consumes `/cube_poses`, tracks cubes over time, and marks tracks `CONFIRMED` after enough hits.
4. While confirmed cube count is below target, orchestration keeps frontier exploration active via `/frontier/start`.
5. `frontier_service_node` reads `/map`, extracts frontier candidates, chooses a goal, and sends `NavigateToPose` action goals.
6. Nav2 publishes motion commands to `/cmd_vel` while navigating.
7. When confirmed count reaches target, orchestration calls `/frontier/stop`, then publishes zero `/cmd_vel` (`SafeStop`).
8. `cmd_vel_to_drive` converts Twist commands to skid-steer wheel velocities and publishes to wheel controller commands.

## Important Runtime Note

Multiple sources can publish to `/cmd_vel` (chase controller, Nav2, orchestrator safe stop). There is no explicit arbiter/mux in this repo, so whichever publisher writes latest controls the robot at that instant.

## Key Files

- `src/cube_detection/cube_detection/cube_detector_node.py`
- `src/cube_detection/cube_detection/cube_chase_controller.py`
- `src/cube_detection/launch/cube_detection.launch.py`
- `src/mks_orchestration/HighOrchestrationNode.py`
- `src/mks_orchestration/FrontierServiceNode.py`
- `src/mks_orchestration/launch/autonomy_orchestration.launch.py`
- `src/mks_control/src/cmd_vel_to_drive.cpp`
- `src/mks_navigation/launch/nav2_bringup.launch.py`

## Bring-up Order (Typical)

```bash
# Terminal A: navigation stack
ros2 launch mks_navigation nav2_bringup.launch.py

# Terminal B: perception + reactive controller
ros2 launch cube_detection cube_detection.launch.py model_path:=/path/to/model.pt

# Terminal C: mission orchestration
ros2 launch mks_orchestration autonomy_orchestration.launch.py
```

