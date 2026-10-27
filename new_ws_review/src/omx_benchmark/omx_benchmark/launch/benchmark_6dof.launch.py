from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch arguments
    num_goals_arg = DeclareLaunchArgument(
        'num_goals',
        default_value='20',
        description='Number of goal configurations'
    )
    
    trials_per_goal_arg = DeclareLaunchArgument(
        'trials_per_goal',
        default_value='10',
        description='Number of trials per goal'
    )
    
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value='/tmp/planning_6dof.db',
        description='Path to results database'
    )
    
    goals_file_arg = DeclareLaunchArgument(
        'goals_file',
        default_value='/tmp/goals_6dof.json',
        description='Path to goals file'
    )
    
    planning_time_arg = DeclareLaunchArgument(
        'planning_time',
        default_value='10.0',
        description='Planning timeout in seconds'
    )
    
    seed_arg = DeclareLaunchArgument(
        'seed',
        default_value='42',
        description='Random seed'
    )
    
    # Path to 6-DOF demo launch
    demo_6dof_path = os.path.expanduser('~/new_ws/src/omx_6dof_moveit/launch/demo.launch.py')
    
    return LaunchDescription([
        num_goals_arg,
        trials_per_goal_arg,
        db_path_arg,
        goals_file_arg,
        planning_time_arg,
        seed_arg,
        
        # Launch 6-DOF robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_6dof_path)
        ),
        
        # Wait for robot to initialize
        TimerAction(
            period=20.0,
            actions=[
                # Generate task-space goals (now using task_goal_generator)
                Node(
                    package='omx_benchmark',
                    executable='task_goal_generator',
                    name='task_goal_generator',
                    output='screen',
                    parameters=[{
                        'num_goals': LaunchConfiguration('num_goals'),
                        'output_file_6dof': LaunchConfiguration('goals_file'),
                        'seed': LaunchConfiguration('seed')
                    }]
                ),
            ]
        ),
        
        # Build scene
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='omx_benchmark',
                    executable='scene_builder',
                    name='scene_builder',
                    output='screen'
                ),
            ]
        ),
        
        # Run benchmark
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package='omx_benchmark',
                    executable='planning_benchmark_runner',
                    name='planning_benchmark_runner',
                    output='screen',
                    parameters=[{
                        'config_type': '6dof',
                        'goals_file': LaunchConfiguration('goals_file'),
                        'trials_per_goal': LaunchConfiguration('trials_per_goal'),
                        'db_path': LaunchConfiguration('db_path'),
                        'planning_time': LaunchConfiguration('planning_time'),
                        'seed': LaunchConfiguration('seed')
                    }]
                ),
            ]
        ),
    ])