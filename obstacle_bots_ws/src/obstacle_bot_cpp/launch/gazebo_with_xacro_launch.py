import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    robotXacroName = 'obstacle_bot'
    
    # this is the name of our package, at the same time this is the name of the 
    # folder that will be used to define the paths
    namePackage = 'obstacle_bot_cpp'
    
    modelFileRelativePath = 'model/obstacle_bot.xacro'
    worldRelativePath = 'model/empty_world.world'
    
    # absolute paths of the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # absolute path of the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldRelativePath)
    
    # get the robot robotDescription from the xacro file
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,launch_arguments={'world': pathWorldFile}.items()
    )
    
    # node to spawn the robot
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robotXacroName,
                   '-topic', 'robot_description'],
        output='screen'
    )
    
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # create an empty launch description object
    launchDescriptionObject = LaunchDescription()
    # add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)
    # add other nodes and actions
    launchDescriptionObject.add_action(spawnModelNode)

    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject