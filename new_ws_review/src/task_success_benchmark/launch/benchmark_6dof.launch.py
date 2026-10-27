from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch arguments
    trials_arg = DeclareLaunchArgument(
        'trials',
        default_value='50',
        description='Number of trials per condition'
    )
    
    # Path to 6-DOF demo launch
    demo_6dof_path = os.path.expanduser('~/new_ws/src/omx_6dof_moveit/launch/demo.launch.py')
    
    return LaunchDescription([
        trials_arg,
        
        # Launch 6-DOF robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_6dof_path)
        ),
        
        # Wait for robot to initialize
        TimerAction(
            period=15.0,
            actions=[
                # Run benchmark
                Node(
                    package='task_success_benchmark',
                    executable='run_task_benchmark',
                    name='task_benchmark',
                    output='screen',
                    arguments=['--skip-5dof', '--trials', LaunchConfiguration('trials')]
                ),
            ]
        ),
    ])
