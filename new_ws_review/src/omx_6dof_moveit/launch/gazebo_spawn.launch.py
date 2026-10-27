from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get URDF via xacro - wrap in ParameterValue with string type
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('omx_6dof_moveit'),
                'config', 'open_manipulator_x.urdf.xacro'
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        
        # Spawn robot in Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'spawn',
                '-robot', '-name', 'open_manipulator_x',
                '-topic', '/robot_description'
            ],
            output='screen'
        )
    ])