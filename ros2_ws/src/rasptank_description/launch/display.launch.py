"""Ver el modelo del RaspTank en RViz y mover sus articulaciones con deslizadores."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rasptank_description')
    urdf = PathJoinSubstitution([share, 'urdf', 'rasptank.urdf.xacro'])
    description = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Deslizadores para mover las articulaciones'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'display.rviz'])]),
    ])
