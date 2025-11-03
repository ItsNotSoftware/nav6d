# nav6d

`nav6d` is a ROS 2 package targeting full **6-DoF local navigation**.

> ⚠️ Work in progress — expect interfaces and parameters to evolve as additional planning and control stages are added.


## Roadmap

* [x] A* path planning over OctoMap voxel grids (position only)
* [x] SLERP-based orientation planning along the path
* [ ] PID controller for trajectory tracking


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

## Launching the Planner

Use the provided launch file:

```bash
ros2 launch nav6d n6d_planner.launch.py
```

Configuration parameters are loaded from `config/n6d_planner.yaml`.


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
| `slerp_orientation`   | SLERP interpolate start→goal orientation  | `false`                       |
| `debug_markers`       | Enable RViz path visualization                 | `true`                        |
| `marker_topic`        | MarkerArray topic name                    | `/nav6d/planner/path_markers` |

Tune these parameters to match your robot geometry, map resolution, and search performance requirements.
The bundled `n6d_planner.yaml` uses the `/**:` wildcard so the same values apply whether you launch the node directly (`ros2 run`) or via `ros2 launch` with a namespace.

> **Performance tip:** OctoMap resolution has a noticeable impact on planning speed—finer grids explode the number of voxels the A* search and collision checks must touch. In our tests a 0.2 m resolution offered a good trade-off between fidelity and runtime; use coarser maps if you need faster replans.

## Example Usage

1. Start an OctoMap server (or another compatible map publisher):

   ```bash
   ros2 run octomap_server octomap_server_node map:=/octomap_full
   ```

2. Launch the planner:

   ```bash
   ros2 launch nav6d n6d_planner.launch.py
   ```

3. Publish a goal to trigger path planning:

   ```bash
   ros2 topic pub --once /nav6d/goal geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'map'},
     pose: {position: {x: 2.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}
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


## Next Steps

Planned additions:

* SLERP-based orientation interpolation along the path
* PID trajectory tracking and closed-loop 6-DoF control
* Unified planner–controller integration


## License

This project is released under the MIT License.
