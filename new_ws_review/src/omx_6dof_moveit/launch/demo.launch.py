import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder  # ← THIS WAS MISSING

def generate_launch_description():

    pkg_share = FindPackageShare("omx_6dof_moveit")

    # ----------------------------------------------------------
    # Generate robot description ONCE and reuse it
    # ----------------------------------------------------------
    urdf_file = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.urdf.xacro"])
    
    # Generate the URDF string
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ", urdf_file,
        " prefix:=''",
        " initial_positions_file:=",
        PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"]),
    ])
    
    # Create the parameter dictionary that ALL nodes will use
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ----------------------------------------------------------
    # Load SRDF
    # ----------------------------------------------------------
    srdf_file = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.srdf"])
    robot_description_semantic_content = Command([
        FindExecutable(name="xacro"), " ", srdf_file
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    
    # ----------------------------------------------------------
    # Load ompl_planning.yaml (create this file if not exists)
    # ----------------------------------------------------------
    # ompl_file = PathJoinSubstitution([pkg_share, "config", "ompl_planning.yaml"])

    # ----------------------------------------------------------
    # Robot State Publisher
    # ----------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ----------------------------------------------------------
    # ros2_control node - MUST have robot_description
    # ----------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,  # CRITICAL: Same robot_description
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])
        ],
        output="screen",
    )

    # ----------------------------------------------------------
    # MoveGroup node - Use MoveItConfigsBuilder but ensure robot_description matches
    # ----------------------------------------------------------
    # First create the moveit_config with all the paths
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="omx_6dof_moveit")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Get the base parameters from moveit_config
    move_group_params = moveit_config.to_dict()
    
    # OVERRIDE robot_description with our version to ensure consistency
    move_group_params["robot_description"] = robot_description["robot_description"]
    
    # ========== CRITICAL FIX: Add joint_states subscription ==========
    move_group_params.update({
        # Joint states configuration - THIS IS WHAT YOU WERE MISSING
        "joint_states": {
            "topic": "/joint_states",
            "queue_size": 10
        },
        
        # Planning scene monitor configuration
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/attached_collision_object",
            "publish_planning_scene_topic": "/planning_scene",
            "monitored_planning_scene_topic": "/monitored_planning_scene",
            "wait_for_initial_state_timeout": 20.0
        },
        
        # Controller manager settings
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
        "use_controller_manager": True,
        
        # Warehouse settings
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("/home/ubuntu/.ros/omx_warehouse.sqlite"),
        
        # General settings
        "use_sim_time": False,
    })
    
    # Add kinematics and ompl parameters
    # move_group_params["ompl"] = ompl_file
    
    delayed_move_group = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[move_group_params],
            )
        ]
    )

    # ----------------------------------------------------------
    # RViz node - Now that core is working, add it back
    # ----------------------------------------------------------
    delayed_rviz = TimerAction(
        period=16.0,
        actions = [Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([pkg_share, "config", "moveit.rviz"])],
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {
                "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                "warehouse_host": os.path.expanduser("/home/ubuntu/.ros/omx_warehouse.sqlite"),
            },
            ],
          )
        ]
    )

    # ----------------------------------------------------------
    # Controller spawners with delays
    # ----------------------------------------------------------
    spawn_joint_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )
    
    spawn_arm_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )
    
    spawn_gripper_controller = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        spawn_joint_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
        delayed_move_group,  
        delayed_rviz, 
    ])