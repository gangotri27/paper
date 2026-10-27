from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get paths to config files
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare('omx_6dof_moveit'),
        'config', 'kinematics.yaml'
    ])
    
    ompl_planning_yaml = PathJoinSubstitution([
        FindPackageShare('omx_6dof_moveit'),
        'config', 'ompl_planning.yaml'
    ])

    return LaunchDescription([
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                kinematics_yaml,
                ompl_planning_yaml,
                {
                    'use_sim_time': True,
                    'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                    'moveit_simple_controller_manager': {
                        'controller_names': []
                    }
                }
            ]
        )
    ])