import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    omx_6dof_moveit_dir = FindPackageShare('omx_6dof_moveit')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # URDF file
    robot_urdf = PathJoinSubstitution([omx_6dof_moveit_dir, 'urdf', 'omx_6dof.urdf'])
    
    # Warehouse database path
    db_path = '/tmp/omx_6dof_warehouse.db'
    
    # MoveGroup node with warehouse settings (NO separate warehouse node!)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': use_sim_time,
            # Warehouse settings - these are handled by move_group itself
            'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
            'warehouse_host': 'localhost',
            'warehouse_port': 33829,
            'warehouse_database_path': db_path
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time'),
        move_group_node,
    ])
