import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_share = FindPackageShare("openmanipulator_x_moveit_config")

    # 1. Load MoveIt Configuration using the Builder
    # This handles URDF, SRDF, Kinematics, etc. automatically
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="openmanipulator_x_moveit_config")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. EXPLICIT SQLite configuration
    # We define this as a dictionary to override any internal defaults
    warehouse_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::SQLiteDatabase",
        "warehouse_host": "/home/ubuntu/.ros/omx_warehouse.sqlite",
        "warehouse_port": 0,
        "warehouse_database": "/home/ubuntu/.ros/omx_warehouse.sqlite"
    }

    # 3. ROBOT STATE PUBLISHER
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    # 4. CONTROLLER MANAGER
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])
        ],
        output="screen",
    )

    # 5. MOVE GROUP (Crucial: must have warehouse_config)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_config,
            {"use_sim_time": True}
        ],
    )

    # 6. RVIZ
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_share, "config", "moveit.rviz"]),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            warehouse_config,
            {"use_sim_time": True}
        ],
    )   

    # 7. SPAWNERS
    spawners = []
    for controller in ["joint_state_broadcaster", "arm_controller"]:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        run_move_group_node,
        rviz_node,
    ] + spawners)