# obstacle_bot_cpp

ROS 2 package providing a URDF description and RViz launch for the custom "obstacle_bot" used in the obstacle-bots ROS2 simulation workspace.

This package contains the robot description (URDF + meshes), an RViz display configuration, and a convenience launch file to publish the `robot_description` and open RViz and the joint state publisher GUI.

## Contents

- `urdf/obstacle_bot.urdf` — Robot description (links, joints, mesh references).
- `meshes/` — STL mesh files referenced by the URDF.
- `config/display.rviz` — RViz configuration to visualize the robot model and TFs.
- `launch/display.launch.py` — Launch file that runs `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.
- `CMakeLists.txt`, `package.xml` — ROS 2 package metadata and install rules.

## Goals / Use cases

- Visualize the robot model and TF frames in RViz.
- Manually manipulate joint positions using the joint state publisher GUI.
- (Optional) Spawn the robot into a Gazebo / Ignition Gazebo world using the `robot_description` topic.

## Prerequisites

- A ROS 2 distribution (this README assumes ROS 2 Humble or later). If you use a different distro, substitute the distro name in the commands below.
- colcon build (for building the workspace).
- `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2` packages (usually available from ROS 2 desktop installs).
- (Optional) `gazebo` / `gazebo_ros` or `ros_gz` packages if you want to spawn the robot into a simulator.

Update the package metadata (`package.xml`) to set an appropriate license and maintainer email if required.

## Build (from workspace root)

This package lives inside the workspace at `obstacle_bots_ws`. From the workspace root run:

```bash
# from repository root (adjust path if needed)
cd obstacle_bot_ros2_development/obstacle_bots_ws
source /opt/ros/humble/setup.bash   # replace 'humble' with your ROS 2 distro
colcon build --symlink-install
```

After a successful build, source the install overlay:

```bash
source install/setup.bash
```

## Run / Visualize

1. Visualize in RViz (recommended for quick checks):

```bash
ros2 launch obstacle_bot_cpp display.launch.py
```

This launch file will:

- Publish `robot_description` via `robot_state_publisher` (so RViz can display the robot).
- Start `joint_state_publisher_gui` so you can move continuous joints (wheels) with sliders.
- Start `rviz2` using the bundled `config/display.rviz` (if present).

2. (Optional) Spawn in Gazebo / Ignition Gazebo

If you want to spawn the robot into a running Gazebo / Ignition Gazebo instance, do the following after launching a simulator world (for example: `ros2 launch gazebo_ros gazebo.launch.py` or your preferred world):

```bash
# spawn from the robot_description topic (requires gazebo_ros spawn_entity.py)
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity obstacle_bot -x 0 -y 0 -z 0
```

Notes:

- The exact spawn command may vary by simulator (`gazebo_ros` vs `ros_gz` bridge). Make sure the appropriate simulator bridge package is installed and a world is running.

## Package structure

```
obstacle_bot_cpp/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── display.launch.py
├── config/
│   └── display.rviz
├── urdf/
│   └── obstacle_bot.urdf
└── meshes/
    ├── Base.STL
    ├── Roda_L.STL
    └── Roda_R.STL
```

## Common issues & troubleshooting

- RViz shows an empty scene / no robot:

  - Ensure `robot_state_publisher` is running and publishing the `robot_description` parameter.
  - Confirm you sourced the workspace `install/setup.bash`.
  - Confirm RViz Fixed Frame is set to `world` (the provided RViz config uses `world`).

- Gazebo spawn fails:
  - Make sure the simulator is running and `gazebo_ros` (or `ros_gz`) is installed.
  - If `spawn_entity.py` is not available, install `gazebo_ros_pkgs` for your distro or use the appropriate ros_gz spawn utilities.

## Development notes

- The URDF was exported from SolidWorks. If you change meshes or the URDF, update the `install(DIRECTORY ...)` entries in `CMakeLists.txt` if you move or rename files.


If you'd like, I can also add a top-level README for the whole `obstacle_bot_ros2_development` workspace that explains how the ROS workspace integrates with the rest of your project (simulator, controllers, nodes), and include example commands for Gazebo+ros_gz_bridge. Tell me which ROS distro you use and whether you use Gazebo Classic or Gazebo/Robot Gazebo (ros_gz).
