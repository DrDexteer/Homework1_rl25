from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    
    declared_arguments = []



    declared_arguments.append(
        DeclareLaunchArgument(
            "jsp_gui", 
            default_value='false',
            description='Flag to enable joint_state_publisher_gui',
            choices=['true', 'false']
        )
    )


    arm_description_path = os.path.join(
        get_package_share_directory('armando_description'))

    xacro_path = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")

    rviz_config = os.path.join(arm_description_path, "config", "rviz", "rviz_config.rviz")

    robot_description_arm_xacro = {"robot_description": Command(['xacro ', xacro_path])}

    


    
    nodes_to_start = []

    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_arm_xacro,
                    {"use_sim_time": True},
            ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
        )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )
    

    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher,
        joint_state_publisher_gui,  
        rviz_node
    ]
        
    return LaunchDescription(declared_arguments + nodes_to_start) 