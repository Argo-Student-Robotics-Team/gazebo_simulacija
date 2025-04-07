import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros 
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='urdf_humble_test').find('urdf_humble_test')
    default_model_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    rvizConfigPath = os.path.join(pkg_share, 'config/config.rviz')

    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[default_model_path]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[default_model_path],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    '''
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity','sasija_model','-topic','robot_description'],
        output='screen'
    )

    pokusaj = launch_ros.actions.Node(
    package="gazebo_ros",
    executable="gzserver",
    output="screen",
    arguments=["--verbose", "-s", "libgazebo_ros_factory.so"]
    ) '''
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )
    
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui',default_value='True', description='Absolute path to robot urdf file'),
        
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])