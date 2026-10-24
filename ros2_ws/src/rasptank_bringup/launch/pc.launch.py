"""PC del alumno con el RaspTank real: detector, misión o rutas, y RViz.

    ros2 launch rasptank_bringup pc.launch.py
    ros2 launch rasptank_bringup pc.launch.py mission:=true

El robot (robot.launch.py en la Raspberry) tiene que estar en el mismo
ROS_DOMAIN_ID y en la misma red.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rasptank_bringup')
    config = lambda name: PathJoinSubstitution([share, 'config', name])  # noqa: E731
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('mission', default_value='false'),
        DeclareLaunchArgument('route', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='false'),
        Node(package='rasptank_perception', executable='detector_node', name='detector_node',
             parameters=[config('perception.yaml')], condition=IfCondition(arg('detector'))),
        Node(package='rasptank_mission', executable='mission_node', name='mission_node',
             parameters=[config('mission.yaml'), {'autostart': arg('autostart')}],
             condition=IfCondition(arg('mission'))),
        Node(package='rasptank_mission', executable='route_node', name='route_node',
             parameters=[config('mission.yaml'), {'autostart': arg('autostart')}],
             condition=IfCondition(arg('route'))),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'rasptank.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
