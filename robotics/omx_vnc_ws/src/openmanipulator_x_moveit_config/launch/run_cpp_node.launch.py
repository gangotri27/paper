import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Define the path EXPLICITLY (Same as your demo.launch.py)
    pkg_share = FindPackageShare("openmanipulator_x_moveit_config")
    urdf_file_path = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.urdf.xacro"])

    # 2. Process the Xacro file manually to get the URDF
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", urdf_file_path,
        " initial_positions_file:=", 
        PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"])
    ])
    
    robot_description = {"robot_description": robot_description_content}

    # 3. Load the SRDF and other configs using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="openmanipulator_x_moveit_config")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # 4. Run the C++ Node with the parameters
    return LaunchDescription([
        Node(
            package="omx_moveit_demo", # Your C++ package
            executable="simple_move",  # Your C++ executable
            name="omx_moveit_demo_node",
            output="screen",
            parameters=[
                robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])