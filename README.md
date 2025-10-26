# PeraSwarm — obstacle botros2 development 

<img width="1920" height="1173" alt="Image" src="https://github.com/user-attachments/assets/4c3434d5-e837-4755-a98c-c113528e859a" />

This workspace contains a ROS 2 package and assets for a custom obstacle robot used in a simulation and visualization environment.

Summary

- Workspace folder: `obstacle_bot_ros2_development/obstacle_bots_ws`
- Key package: `obstacle_bots_ws/src/obstacle_bot_cpp` — contains URDF, meshes, RViz config and a launch file to publish the robot description and open RViz.

Quick assumptions

- This README assumes you use a ROS 2 desktop distribution such as `humble` or later and have `colcon` installed. If you use a different distro, replace the distro name in the `source` commands below.
- Simulator: instructions include both Gazebo Classic (`gazebo_ros`) and the newer ros_gz bridge (`ros_gz`) patterns where appropriate — adapt to your simulator of choice.

Repository layout (relevant parts)

```
obstacle_bot_ros2_development/
├── obstacle_bots_ws/                 # ROS 2 workspace
│   ├── src/
│   │   └── obstacle_bot_cpp/         # package with URDF, meshes, launch, rviz config
│   │       ├── launch/display.launch.py
│   │       ├── urdf/obstacle_bot.urdf
│   │       ├── meshes/*.STL
│   │       └── config/display.rviz
├── 3DModel/                           # CAD and exported assets
├── simulator/                         # python simulator tools and helpers
└── ...
```

Build the ROS workspace

1. Open a terminal and go to the workspace root:

```bash
cd ...../obstacle_bot_ros2_development/obstacle_bots_ws
```

2. Source your ROS 2 distro and build:

```bash
source /opt/ros/humble/setup.bash   # replace 'humble' with your ROS 2 distro
colcon build --symlink-install
```

3. Source the overlay after building:

```bash
source install/setup.bash
```

Visualize the robot (RViz)

After sourcing the overlay, launch the provided RViz + robot_state_publisher combination:

```bash
ros2 launch obstacle_bot_cpp display.launch.py
```

This will:

- Start `robot_state_publisher` and publish `robot_description`.
- Open `joint_state_publisher_gui` so you can manipulate the continuous joints (wheels).
- Launch `rviz2` with the provided `config/display.rviz` (if present).

Spawn into Gazebo (optional)

If you want to spawn the robot in a simulator you must first start a simulator world. Examples:

- Gazebo Classic (gazebo_ros):

```bash
# in one terminal: start a gazebo world (example)
ros2 launch gazebo_ros gazebo.launch.py

# in another terminal (after sourcing):
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity obstacle_bot -x 0 -y 0 -z 0
```

- Ignition / Gazebo (ros_gz bridge): the exact spawn utility may differ. For ros_gz, consult `ros_gz` documentation or use a small node that reads `/robot_description` and calls the appropriate spawn service.

Notes & troubleshooting

- If RViz shows nothing, confirm you sourced `install/setup.bash` and that `robot_state_publisher` is running.
- RViz Fixed Frame in the provided config is `world`. If you change the URDF base frame, update RViz or the URDF accordingly.
- If meshes do not render, ensure the URDF uses `package://obstacle_bot_cpp/meshes/...` paths and the package is installed (`colcon build` + `source`).

Development

- If you add runtime dependencies, update `package.xml` and `CMakeLists.txt`.
