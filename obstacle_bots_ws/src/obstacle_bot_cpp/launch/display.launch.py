import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_share_dir = get_package_share_directory('obstacle_bot_cpp')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share_dir, 'model', 'obstacle_bot.urdf')
    
    # Read URDF contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher node - this will publish robot_description parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 15.0
        }]
    )
    
    # Joint State Publisher GUI node - provides sliders to control joint positions
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share_dir, 'config', 'display.rviz')] if os.path.exists(os.path.join(pkg_share_dir, 'config', 'display.rviz')) else []
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])