# nav6d

`nav6d` is a ROS 2 package targeting full **6-DoF local navigation**, from collision-aware path planning in 3D to PD-based trajectory tracking.


## Index

- [Overview](#overview)
- [Features](#features)
- [Getting Started](#getting-started)
- [Dependencies](#dependencies)
- [Launching nav6d](#launching-nav6d)
- [Runtime Requirements](#runtime-requirements)
- [Configuration Highlights](#configuration-highlights)
- [Example Usage](#example-usage)
- [Node Overview](#node-overview)
- [Next Steps](#next-steps)
- [License](#license)


## Overview

At a high level, `nav6d` assumes:

- A 3D occupancy map from OctoMap 
- A pose estimate for the robot body frame
- Goals expressed as `PoseStamped` in the map frame on `/nav6d/goal`

## Node Overview

The package is split into a planner, two controller variants, and a path evaluator:

- `n6d_planner`: consumes the OctoMap and robot pose, then computes a collision-free 3D path (`nav_msgs/Path`) with consistent orientations along the way.
- `n6d_velocity_controller`: consumes the planned path, current pose, and IMU; it projects the robot onto the path, selects a lookahead “carrot” pose, runs a 6‑DoF PD law, and publishes body-frame velocity commands (`geometry_msgs/Twist`).
- `n6d_force_controller`: shares the same control core but publishes wrench commands (`geometry_msgs/Wrench`) for actuators that expect force/torque inputs.
- `n6d_path_evaluator`: scores any `nav_msgs/Path` against the OctoMap and publishes `nav6d/msg/PathQuality` on `/nav6d/path_quality`.

Planner and the selected controller are typically launched together via `n6d.launch.py`, but you can also run only the planner or only a controller.


## Features

* A* path planning over OctoMap voxel grids (position only)
* SLERP-based orientation planning along the path
* 6-DoF PD controller for trajectory tracking (position + attitude)
* Planner-agnostic path quality scoring against OctoMap geometry


## Getting Started

Clone this repository (or add it as a submodule) inside the `src/` directory of your ROS 2 workspace, then build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select nav6d
```

## Dependencies

### Required

`nav6d` requires the OctoMap message definitions:

```bash
sudo apt install ros-${ROS_DISTRO}-octomap-msgs
```

An active OctoMap is also required, but it can be published by **any** package that provides an `octomap_msgs/msg/Octomap` topic.

The path evaluator publishes `nav6d/msg/PathQuality`, which is defined in the `nav6d` package.

### Recommended

For testing, the standard OctoMap packages work out of the box and are recommended:

```bash
sudo apt install ros-${ROS_DISTRO}-octomap ros-${ROS_DISTRO}-octomap-ros ros-${ROS_DISTRO}-octomap-server
```

The `octomap_server` node bundled with `octomap_server` has been successfully tested with `nav6d` and is suggested as the default `/octomap_full` provider.

## Launching nav6d

You can launch planner and controller together, or each component independently.

**Planner + controller (recommended):**

```bash
ros2 launch nav6d n6d.launch.py
```

This starts `n6d_planner` plus the velocity controller (`controller_type:=velocity` by default).
Switch to the force-based controller with:

```bash
ros2 launch nav6d n6d.launch.py controller_type:=force
```

**Planner + controller + optional path evaluator:**

```bash
ros2 launch nav6d n6d.launch.py enable_path_evaluator:=true
```

**Planner only:**

```bash
ros2 launch nav6d n6d_planner.launch.py
```

**Controller only:**

```bash
ros2 launch nav6d n6d_controller.launch.py
```

Pass `controller_type:=force` to launch only the wrench-based controller.

**Path evaluator only:**

```bash
ros2 launch nav6d n6d_path_evaluator.launch.py
```

## Runtime Requirements

### Subscribed Topics

**Note:** Topic names are configurable; see the YAML files in `config/`.

| Type                            | Description                                  |
| :------------------------------ | :------------------------------------------- |
| `octomap_msgs/msg/Octomap`      | 3D occupancy map used for collision checking |
| `geometry_msgs/msg/PoseStamped` | Robot pose in the map frame                  |
| `geometry_msgs/msg/PoseStamped` | Target pose to plan toward                   |

### Published Topics

| Topic                         | Type                                 | Description                          |
| :---------------------------- | :----------------------------------- | :----------------------------------- |
| `/nav6d/planner/path`         | `nav_msgs/msg/Path`                  | Generated waypoint path              |
| `/nav6d/planner/path_markers` | `visualization_msgs/msg/MarkerArray` | Debug visualization markers for RViz |
| `/nav6d/path_quality`         | `nav6d/msg/PathQuality`              | Path quality scores from evaluator   |
| `/space_cobot/cmd_vel`        | `geometry_msgs/msg/Twist`            | Velocity controller output           |
| `/space_cobot/cmd_force`      | `geometry_msgs/msg/Wrench`           | Force controller output              |

Each newly received goal triggers a replanning pass.
Markers are only published if `debug_markers` is enabled.


## Configuration Highlights

Parameters are managed via the YAML configuration file.

| Parameter             | Description                               |
| :-------------------- | :---------------------------------------- |
| `map_topic`           | OctoMap input topic                       |
| `pose_topic`          | Robot pose topic                          |
| `goal_topic`          | Goal pose topic                           |
| `path_topic`          | Planned path output topic                 |
| `map_frame`           | Frame ID for path poses                   |
| `robot_radius`        | Collision model radius (m)                |
| `occupancy_threshold` | Probability threshold for occupied voxels |
| `max_search_range`    | Maximum search distance (m)               |
| `max_expansions`      | A* node expansion limit                   |
| `line_sample_step`    | Step size for line feasibility checks (m) |
| `slerp_orientation`   | SLERP interpolate start→goal orientation  |
| `debug_markers`       | Enable RViz path visualization            |
| `marker_topic`        | MarkerArray topic name                    |

Additional controller-specific parameters from `config/n6d_force_controller.yaml`
(the velocity controller uses the same keys but publishes twists instead of wrenches):

| Parameter                    | Description                                        |
| :--------------------------- | :------------------------------------------------- |
| `cmd_force_topic` / `cmd_velocity_topic`| Output topic for wrench or twist commands       |
| `control_rate_hz`           | PD loop frequency (Hz)                             |
| `lookahead_distance`        | "Carrot" distance along path (m)                   |
| `path_reacquire_period`     | Projection refresh period (s)                      |
| `feedforward_speed`         | Tangential feedforward speed (m/s)                 |
| `approach_slowdown_distance`| Distance where lookahead/feedforward are reduced   |
| `velocity_ema_alpha`        | EMA blend for velocity estimation                  |
| `pos_tolerance`             | Goal position tolerance (m)                        |
| `orientation_tolerance_rad` | Goal orientation tolerance (rad)                   |
| `max_velocity_mps`          | Max allowed linear speed (m/s) before braking      |
| `velocity_brake_gain`       | Gain for velocity-based braking                    |
| `use_goal_orientation`      | If true, track final goal orientation explicitly   |
| `kp_linear`                 | XYZ position PD gains (`[Kpx, Kpy, Kpz]`)          |
| `kd_linear`                 | XYZ velocity PD gains (`[Kdx, Kdy, Kdz]`)          |
| `kp_angular`                | Roll/pitch/yaw attitude gains                      |
| `kd_angular`                | Roll/pitch/yaw angular velocity gains              |
| `max_force_xyz` / `max_linear_velocity_xyz`             | Per-axis body-frame command clamp (linear)  |
| `max_torque_rpy` / `max_angular_velocity_rpy`            | Per-axis body-frame command clamp (angular) |
| `debug_enabled`             | Enable verbose logs and debug topics               |
| `debug_speed_topic`         | Topic for scalar linear speed debug                |
| `debug_projected_pose_topic`| Topic for projected-on-path pose                   |
| `debug_target_pose_topic`   | Topic for lookahead "carrot" pose                  |
| `debug_error_topic`         | Topic for pose/orientation error debug             |

Path quality parameters from `config/n6d_path_evaluator.yaml`:

| Parameter             | Description                                 |
| :-------------------- | :------------------------------------------ |
| `robot_radius`        | Collision radius for clearance checks (m)   |
| `alpha`               | Clearance scale factor (T = alpha * radius) |
| `occupancy_threshold` | Probability threshold for occupied voxels   |
| `sample_step`         | Min spacing between sampled poses (m)       |
| `w_c` / `w_n` / `w_t` / `w_e` | Weights for clearance, narrow, turn, efficiency |

Tune these parameters to match your robot geometry, map resolution, and search performance requirements.
`yaw_tolerance_rad` is still accepted for legacy configs, but `orientation_tolerance_rad` is preferred.
The bundled `n6d_planner.yaml` uses the `/**:` wildcard so the same values apply whether you launch the node directly (`ros2 run`) or via `ros2 launch` with a namespace.

> **Performance tip:** OctoMap resolution has a noticeable impact on planning speed—finer grids explode the number of voxels the A* search and collision checks must touch. In our tests a 0.2 m resolution offered a good trade-off between fidelity and runtime; use coarser maps if you need faster replans.




## Example Usage

1. Start an OctoMap server (or another compatible map publisher):

   ```bash
   ros2 run octomap_server octomap_server_node map:=/octomap_full
   ```

2. Launch planner and controller together (recommended):

   ```bash
   ros2 launch nav6d n6d.launch.py
   ```
   Append `controller_type:=force` if you need wrench (force/torque) outputs instead of velocity commands.

3. Publish a goal to trigger path planning and closed-loop tracking.  
   The example below targets a pose at `(2, 0, 1)` with a 90° yaw about +Z:

   ```bash
   ros2 topic pub --once /nav6d/goal geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'map'},
     pose: {
       position: {x: 2.0, y: 0.0, z: 1.0},
       orientation: {x: 0.3799282, y: 0.5963678, z: 0.3799282, w: 0.5963678}
     }
   }"
   ```

4. Visualize the result in RViz by adding:

   * A `Path` display on `/nav6d/planner/path`
   * A `MarkerArray` display on `/nav6d/planner/path_markers`
   * Fixed frame: `map`
