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
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable



def generate_launch_description():
    
    declared_arguments = []

    arm_description_path = os.path.join(
        get_package_share_directory('armando_description'))

    xacro_armando = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")

    robot_description_arm_xacro = {"robot_description": Command(['xacro ', xacro_armando])}

    rviz_config = os.path.join(arm_description_path, "config", "rviz", "rviz_config.rviz")
    

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui", 
            default_value='true',
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_package",
            description='The package where the robot description is located',
            default_value='armando_description',
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument('urdf_package_path',
            description='The path to the robot description relative to the package root',
            default_value= xacro_armando,
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument('controller_type',
            description='Type of controller to spawn: "position" or "trajectory',
            default_value= 'position',
        )
    )

    nodes_to_start = []

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_arm_xacro,
                    {"use_sim_time": True},
            ],
    )

    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )

    
    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '0.5', '-unpause'],
        output='screen',
    )

 
    ign = [empty_world_launch, urdf_spawner_node]


    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    ) 

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'position'"])) 
    ) 

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],  
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'trajectory'"]))

    ) 

    delay_joint_pos_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node,
            on_exit=[position_controller],
        )
    )

    delay_joint_traj_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node,
            on_exit=[joint_trajectory_controller],
        )
    )
    
    delay_joint_state_broadcaster = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawner_node,
                on_exit=[joint_state_broadcaster],
            )
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )

    
    nodes_to_start = [
        robot_state_publisher_node,
        *ign,
        delay_joint_state_broadcaster,
        delay_joint_pos_controller,
        delay_joint_traj_controller,
        rviz_node,
        bridge_camera
    ]
        
    return LaunchDescription(declared_arguments + nodes_to_start) 