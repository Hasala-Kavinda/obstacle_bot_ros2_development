import os
import re
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_bot_cpp')

    # Path to the robot URDF inside this package
    urdf_file = os.path.join(pkg_share, 'model', 'obstacle_bot.urdf')
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF not found: {urdf_file}")

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Remove XML encoding declaration if present. spawn_entity (lxml) does not accept
    # unicode strings containing an encoding declaration. Strip it so the XML parser
    # accepts the string input.
    robot_description = re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", robot_description, count=1)

    # Include the standard gazebo launch from the gazebo_ros package
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        # pass no extra args by default; user can modify this file as needed
    )

    # Robot State Publisher to publish /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn entity in Gazebo from the /robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'obstacle_bot', '-x', '0', '-y', '0', '-z', '0'],
        output='screen'
    )

    # Delay spawning to give Gazebo time to start
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        delayed_spawn,
    ])
