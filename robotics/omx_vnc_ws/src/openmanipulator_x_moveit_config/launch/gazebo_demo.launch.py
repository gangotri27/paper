import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_share = FindPackageShare("openmanipulator_x_moveit_config")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")
    description_share = FindPackageShare("open_manipulator_description")
    
    # 1. FIX MESH PATHS
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([description_share, '..'])
    )

    # 2. GENERATE ROBOT DESCRIPTION
    urdf_file_path = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.urdf.xacro"])
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", urdf_file_path,
        " initial_positions_file:=", 
        PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"])
    ])
    robot_description = {"robot_description": robot_description_content}

    # 3. LOAD MOVEIT CONFIG
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="openmanipulator_x_moveit_config")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 4. START GAZEBO
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 5. SPAWN THE ROBOT
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'open_manipulator_x',
                   '-allow_renaming', 'true'],
        output='screen'
    )

    # 6. CLOCK BRIDGE (CRITICAL FIX FOR TIMING)
    # This bridges the Gazebo clock to ROS 2 so MoveIt syncs with simulation time
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 7. ROBOT STATE PUBLISHER
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 8. MOVE GROUP
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    # 9. RVIZ
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
            {'use_sim_time': True}
        ],
    )

    # 10. SPAWN CONTROLLERS
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        set_gz_resource_path,
        start_gazebo,
        bridge,  # <--- Added Bridge here
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_spawner, arm_spawner, gripper_spawner, run_move_group_node, rviz_node],
            )
        ),
    ])