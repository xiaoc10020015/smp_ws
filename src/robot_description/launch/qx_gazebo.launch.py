
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'robot_description'
    robot_name_in_model = 'qianxun'
    urdf_name = "qianxun.xacro"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'xacro/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'world/fishbot.world')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(urdf_model_path),
                                      description='Absolute path to robot urdf file')
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),value_type=str)

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')
        
    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,  
            '-topic', '/robot_description'
        ], 
        output='screen'
    )
	
    # Start Robot State publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    ld.add_action(model_arg)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(robot_state_publisher_node)

    return ld