import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Define the path EXPLICITLY
    pkg_share = FindPackageShare("omx_5dof_moveit")
    urdf_file_path = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.urdf.xacro"])

    # 2. Process the Xacro file manually
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", urdf_file_path,
        " initial_positions_file:=", 
        PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"])
    ])
    
    robot_description = {
        "robot_description": ParameterValue(
        robot_description_content,
        value_type=str
    )}

    # 3. Load other MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="omx_5dof_moveit")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 4. Create Nodes

    # --- CRITICAL NODE: ROBOT STATE PUBLISHER ---
    # This is what you were missing. It publishes the /robot_description topic.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])
        ],
        output="screen",
    )

    warehouse_config = {
        "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        "warehouse_host": "moveit_mongo",
        "warehouse_port": 27017,
    }

    # Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
            warehouse_config,
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([pkg_share, "config", "moveit.rviz"])],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            warehouse_config,
        ],
    )

    # Spawners
    spawners = []
    for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
            )
        )

    # Ensure robot_state_publisher_node is in this list!
    nodes_to_start = [robot_state_publisher_node, ros2_control_node, run_move_group_node, rviz_node] + spawners
    return LaunchDescription(nodes_to_start)
