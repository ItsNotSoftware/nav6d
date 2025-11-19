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

- A 3D occupancy map from OctoMap on `/octomap_full`
- A pose estimate for the robot body frame on `/space_cobot/pose`
- Goals expressed as `PoseStamped` in the map frame on `/nav6d/goal`

The package is split into a planner plus two controller variants:

- `n6d_planner`: consumes the OctoMap and robot pose, then computes a collision-free 3D path (`nav_msgs/Path`) with consistent orientations along the way.
- `n6d_velocity_controller`: consumes the planned path, current pose, and IMU; it projects the robot onto the path, selects a lookahead “carrot” pose, runs a 6‑DoF PD law, and publishes body-frame velocity commands (`geometry_msgs/Twist`) to `/space_cobot/cmd_vel`.
- `n6d_force_controller`: shares the same control core but publishes wrench commands (`geometry_msgs/Wrench`) to `/space_cobot/cmd_force` for actuators that expect force/torque inputs.

Planner and the selected controller are typically launched together via `n6d.launch.py`, but you can also run only the planner or only a controller.


## Features

* A* path planning over OctoMap voxel grids (position only)
* SLERP-based orientation planning along the path
* 6-DoF PD controller for trajectory tracking (position + attitude)


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

**Planner only:**

```bash
ros2 launch nav6d n6d_planner.launch.py
```

**Controller only:**

```bash
ros2 launch nav6d n6d_controller.launch.py
```

Pass `controller_type:=force` to launch only the wrench-based controller.

## Runtime Requirements

### Subscribed Topics

**Note:** The topic names listed below are defaults. You can override all of them in `config/n6d_planner.yaml` via `map_topic`, `pose_topic`, `goal_topic`, `path_topic`, and `marker_topic`.

| Topic               | Type                            | Description                                  |
| :------------------ | :------------------------------ | :------------------------------------------- |
| `/octomap_full`     | `octomap_msgs/msg/Octomap`      | 3D occupancy map used for collision checking |
| `/space_cobot/pose` | `geometry_msgs/msg/PoseStamped` | Robot pose in the map frame                  |
| `/nav6d/goal`       | `geometry_msgs/msg/PoseStamped` | Target pose to plan toward                   |

### Published Topics

| Topic                         | Type                                 | Description                          |
| :---------------------------- | :----------------------------------- | :----------------------------------- |
| `/nav6d/planner/path`         | `nav_msgs/msg/Path`                  | Generated waypoint path              |
| `/nav6d/planner/path_markers` | `visualization_msgs/msg/MarkerArray` | Debug visualization markers for RViz |

Each newly received goal triggers a replanning pass.
Markers are only published if `debug_markers` is enabled.


## Configuration Highlights

Parameters are managed via the YAML configuration file.

| Parameter             | Description                               | Default                       |
| :-------------------- | :---------------------------------------- | :---------------------------- |
| `map_topic`           | OctoMap input topic                       | `/octomap_full`               |
| `pose_topic`          | Robot pose topic                          | `/space_cobot/pose`           |
| `goal_topic`          | Goal pose topic                           | `/nav6d/goal`                 |
| `path_topic`          | Planned path output topic                 | `/nav6d/planner/path`         |
| `map_frame`           | Frame ID for path poses                   | `map`                         |
| `robot_radius`        | Collision model radius (m)                | `0.35`                        |
| `occupancy_threshold` | Probability threshold for occupied voxels | `0.5`                         |
| `max_search_range`    | Maximum search distance (m)               | `15.0`                        |
| `max_expansions`      | A* node expansion limit                   | `60000`                       |
| `line_sample_step`    | Step size for line feasibility checks (m) | `0.25`                        |
| `slerp_orientation`   | SLERP interpolate start→goal orientation  | `true`                       |
| `debug_markers`       | Enable RViz path visualization                 | `true`                        |
| `marker_topic`        | MarkerArray topic name                    | `/nav6d/planner/path_markers` |

Additional controller-specific parameters from `config/n6d_force_controller.yaml`
(the velocity controller uses the same keys but publishes twists instead of wrenches):

| Parameter                    | Description                                        | Example Default                            |
| :--------------------------- | :------------------------------------------------- | :----------------------------------------- |
| `cmd_force_topic` / `cmd_velocity_topic`| Output topic for wrench or twist commands       | `/space_cobot/cmd_force` (force) / `/space_cobot/cmd_vel` (velocity) |
| `control_rate_hz`           | PD loop frequency (Hz)                             | `100.0`                                    |
| `lookahead_distance`        | "Carrot" distance along path (m)                   | `0.7`                                      |
| `path_reacquire_period`     | Projection refresh period (s)                      | `0.10`                                     |
| `feedforward_speed`         | Tangential feedforward speed (m/s)                 | `0.0`                                      |
| `approach_slowdown_distance`| Distance where lookahead/feedforward are reduced   | `1.5`                                      |
| `velocity_ema_alpha`        | EMA blend for velocity estimation                  | `0.6`                                      |
| `pos_tolerance`             | Goal position tolerance (m)                        | `0.12`                                     |
| `yaw_tolerance_rad`         | Goal yaw tolerance (rad)                           | `0.015`                                    |
| `max_velocity_mps`          | Max allowed linear speed (m/s) before braking      | `0.4`                                      |
| `velocity_brake_gain`       | Gain for velocity-based braking                    | `3.5`                                      |
| `use_goal_orientation`      | If true, track final goal orientation explicitly   | `false`                                    |
| `kp_linear`                 | XYZ position PD gains (`[Kpx, Kpy, Kpz]`)          | `[14.8, 14.8, 14.0]`                        |
| `kd_linear`                 | XYZ velocity PD gains (`[Kdx, Kdy, Kdz]`)          | `[12.0, 12.0, 12.0]`                        |
| `kp_angular`                | Roll/pitch/yaw attitude gains                      | `[2.0, 2.0, 2.0]`                           |
| `kd_angular`                | Roll/pitch/yaw angular velocity gains              | `[1.0, 1.0, 1.0]`                           |
| `max_force_xyz` / `max_linear_velocity_xyz`             | Per-axis body-frame command clamp (linear)  | `[300.0,300.0,300.0]` (force) / `[0.6,0.6,0.4]` (velocity) |
| `max_torque_rpy` / `max_angular_velocity_rpy`            | Per-axis body-frame command clamp (angular) | `[8.0,8.0,8.0]` (force) / `[0.6,0.6,0.6]` (velocity)       |
| `debug_enabled`             | Enable verbose logs and debug topics               | `true`                                     |
| `debug_speed_topic`         | Topic for scalar linear speed debug                | `/nav6d/force_controller/debug/linear_speed` (`/nav6d/velocity_controller/...` for velocity)     |
| `debug_projected_pose_topic`| Topic for projected-on-path pose                   | `/nav6d/force_controller/debug/path_projection` (force) / `/nav6d/velocity_controller/...` (velocity) |
| `debug_target_pose_topic`   | Topic for lookahead "carrot" pose                  | `/nav6d/force_controller/debug/carrot_pose` (force) / `/nav6d/velocity_controller/...` (velocity)     |
| `debug_error_topic`         | Topic for pose/orientation error debug             | `/nav6d/force_controller/debug/control_error` (force) / `/nav6d/velocity_controller/...` (velocity)   |

Tune these parameters to match your robot geometry, map resolution, and search performance requirements.
The bundled `n6d_planner.yaml` uses the `/**:` wildcard so the same values apply whether you launch the node directly (`ros2 run`) or via `ros2 launch` with a namespace.

> **Performance tip:** OctoMap resolution has a noticeable impact on planning speed—finer grids explode the number of voxels the A* search and collision checks must touch. In our tests a 0.2 m resolution offered a good trade-off between fidelity and runtime; use coarser maps if you need faster replans.

### Controller Configuration

Two YAML files ship with nav6d to keep the controller variants separate:

- `config/n6d_velocity_controller.yaml` contains twist-specific parameters (clamps, feedforward,
  debug topics). This file is selected by default in `n6d.launch.py` and in
  `n6d_controller.launch.py` when `controller_type:=velocity`.
- `config/n6d_force_controller.yaml` mirrors the same structure but publishes wrenches. Launch with
  `controller_type:=force` to load this file automatically.

Both YAMLs expose identical knobs (topics, PD gains, tolerances, debug flags) so you can tune one
controller and mirror the settings to the other when needed. The most important fields to update
are:

1. `cmd_velocity_topic` / `cmd_force_topic`: body-frame output topics consumed by your bridge or
   actuators.
2. `lookahead_distance`, `approach_slowdown_distance`, and `feedforward_speed`: shape how the
   carrot pose is selected along the path.
3. `kp_linear`, `kd_linear`, `kp_angular`, `kd_angular`: PD gains for translation and rotation.
4. `max_linear_velocity_xyz` / `max_force_xyz` and `max_angular_velocity_rpy` / `max_torque_rpy`:
   per-axis clamps that keep the controller outputs within what the vehicle can track.

Because the velocity controller is now the default, make sure downstream bridges (e.g. `cmd_bridge`)
listen on `/space_cobot/cmd_vel`. Switch to `/space_cobot/cmd_force` only when using actuators
that natively consume wrenches.

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


## Node Overview

`n6d_planner` performs:

* Straight-line collision checking between start and goal
* Fallback A* pathfinding in 3D voxel space (26-connected neighborhood)
* Collision inflation using the configured `robot_radius`
* Optional debug visualization in RViz
* Deferred goal and map update handling while planning

Planning performance statistics are printed to the console at each run.

`n6d_velocity_controller` (default) projects the robot onto the generated path, selects a
lookahead pose, closes the loop with a 6-DoF PD law, and publishes body-frame twist commands on
`/space_cobot/cmd_vel`. `n6d_force_controller` shares this logic but publishes wrenches on
`/space_cobot/cmd_force` for setups that expect forces/torques instead of velocities.
