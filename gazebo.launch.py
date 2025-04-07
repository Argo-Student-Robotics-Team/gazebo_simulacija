import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='urdf_humble_test').find('urdf_humble_test')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover_model', '-topic', 'robot_description'],
        output='screen',
        name='spawn_entity'
    )

    gazebo_server = launch.actions.ExecuteProcess(
    cmd=['/usr/bin/gzserver', '--verbose'],
    output='screen',
    name='gazebo_server'
    )   

    gazebo_client = launch.actions.ExecuteProcess(
        cmd=['/usr/bin/gzclient'],
        output='screen',
        name='gazebo_client'
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui', default_value='True', description='Launch Gazebo with GUI'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_entity
    ])
