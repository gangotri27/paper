import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load parameters using the configs builder
    # Note: We point to the 'openmanipulator_x_moveit_config' package we built in Ex 10
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="openmanipulator_x_moveit_config")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="omx_moveit_demos",
            executable="ex11_planning_scene_attach",
            name="ex11_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])