"""
MoveIt2 Launch File for UR5 Assembly with Namespace Support

This launch file starts:
- move_group: MoveIt2 motion planning node
- rviz2: (optional) visualization with MoveIt plugin

Usage:
    # Default namespace (ur5)
    ros2 launch ur5_moveit_config ur_moveit.launch.py
    
    # Custom namespace
    ros2 launch ur5_moveit_config ur_moveit.launch.py robot_namespace:=robot1
    
    # Without RViz
    ros2 launch ur5_moveit_config ur_moveit.launch.py use_rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "ur5_assembly", 
        package_name="ur5_moveit_config"
    ).to_moveit_configs()

    launch_package_path = moveit_config.package_path

    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="ur5",
            description="Namespace for the robot (matches robot_state_publisher namespace)"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with MoveIt plugin"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "allow_trajectory_execution",
            default_value="true",
            description="Allow MoveIt to execute trajectories"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_monitored_planning_scene",
            default_value="true",
            description="Publish the monitored planning scene"
        )
    )

    # Get launch configurations
    robot_namespace = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")
    publish_monitored_planning_scene = LaunchConfiguration("publish_monitored_planning_scene")

    # MoveGroup configuration
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "use_sim_time": use_sim_time,
        "allow_trajectory_execution": allow_trajectory_execution,
        "publish_planning_scene": publish_monitored_planning_scene,
        "publish_geometry_updates": publish_monitored_planning_scene,
        "publish_state_updates": publish_monitored_planning_scene,
        "publish_transforms_updates": publish_monitored_planning_scene,
        "monitor_dynamics": False,
    }

    # Combine all parameters
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # MoveGroup node with namespace
    # Remappings ensure move_group connects to the namespaced robot
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        #namespace=robot_namespace,
        output="screen",
        parameters=move_group_params,
        remappings=[
            # Remap joint_states to namespaced topic
            ("joint_states", "joint_states"),
            # Robot description is published by namespaced robot_state_publisher
            ("robot_description", "robot_description"),
        ],
    )

    # RViz node with MoveIt config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_moveit_config"),
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments + [
            move_group_node,
            rviz_node,
        ]
    )


