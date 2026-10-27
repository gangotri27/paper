from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'db_5dof',
            default_value='/tmp/planning_5dof.db',
            description='5-DOF database path'
        ),
        
        DeclareLaunchArgument(
            'db_6dof',
            default_value='/tmp/planning_6dof.db',
            description='6-DOF database path'
        ),
        
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/planning_analysis',
            description='Output directory'
        ),
        
        Node(
            package='omx_benchmark',
            executable='planning_analyzer.py',
            name='planning_analyzer',
            output='screen',
            parameters=[{
                'db_5dof': LaunchConfiguration('db_5dof'),
                'db_6dof': LaunchConfiguration('db_6dof'),
                'output_dir': LaunchConfiguration('output_dir')
            }]
        ),
    ])