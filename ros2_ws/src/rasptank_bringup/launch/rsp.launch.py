"""robot_state_publisher con el URDF del RaspTank (lo incluyen los demás launch)."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = PathJoinSubstitution([FindPackageShare('rasptank_description'), 'urdf',
                                 'rasptank.urdf.xacro'])
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]),
                                                              value_type=str)}]),
    ])
