from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value='/tmp/omx_benchmark_results.db',
        description='Path to results database'
    )
    
    return LaunchDescription([
        db_path_arg,
        
        # Run statistical analysis
        Node(
            package='omx_benchmark',
            executable='statistical_analyzer',
            name='statistical_analyzer',
            output='screen',
            parameters=[{
                'db_path': LaunchConfiguration('db_path')
            }]
        ),
    ])