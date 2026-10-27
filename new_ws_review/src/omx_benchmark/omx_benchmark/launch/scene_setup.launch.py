from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    return LaunchDescription([
        # Scene builder node
        Node(
            package='omx_benchmark',
            executable='scene_builder',
            name='scene_builder',
            output='screen',
        ),
        
        # Verification command (will run after scene builder)
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/planning_scene', '--once', '--no-arr'],
            output='screen',
            name='scene_verifier'
        ),
    ])