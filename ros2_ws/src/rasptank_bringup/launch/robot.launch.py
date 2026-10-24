"""Nodos del RaspTank real. Se lanza en la Raspberry Pi (ver docker/README.md).

    ros2 launch rasptank_bringup robot.launch.py
    ros2 launch rasptank_bringup robot.launch.py camera_backend:=opencv camera_source:=tcp://127.0.0.1:8554

La visión, la misión y RViz se lanzan en el PC con pc.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rasptank_bringup')
    robot = PathJoinSubstitution([share, 'config', 'robot.yaml'])
    arg = LaunchConfiguration
    cmd_topic = PythonExpression(["'cmd_vel_safe' if '", arg('governor'), "' == 'true' else 'cmd_vel'"])
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_backend', default_value='picamera2',
                              description='picamera2 u opencv'),
        DeclareLaunchArgument('camera_source', default_value='0',
                              description='con opencv: dispositivo o URL del vídeo'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('signals', default_value='true', description='LEDs y zumbador'),
        DeclareLaunchArgument('governor', default_value='true',
                              description='limitar la velocidad con el ultrasonido'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'rsp.launch.py']))),
        # if base_node dies it is started again (and its first act is to stop the
        # motors); meanwhile motor_watchdog switches them off
        Node(package='rasptank_hw', executable='base_node', name='base_node',
             parameters=[robot, {'cmd_topic': cmd_topic}], respawn=True, respawn_delay=1.0),
        Node(package='rasptank_hw', executable='motor_watchdog_node', name='motor_watchdog',
             parameters=[robot], respawn=True, respawn_delay=1.0),
        Node(package='rasptank_hw', executable='governor_node', name='governor',
             parameters=[robot], condition=IfCondition(arg('governor'))),
        Node(package='rasptank_hw', executable='arm_node', name='arm_node',
             parameters=[robot]),
        Node(package='rasptank_hw', executable='signals_node', name='signals_node',
             parameters=[robot], condition=IfCondition(arg('signals'))),
        Node(package='rasptank_hw', executable='sensors_node', name='sensors_node',
             parameters=[robot], condition=IfCondition(arg('sensors'))),
        Node(package='rasptank_hw', executable='camera_node', name='camera_node',
             parameters=[robot, {'backend': arg('camera_backend'),
                                 'source': arg('camera_source')}],
             condition=IfCondition(arg('camera'))),
    ])
