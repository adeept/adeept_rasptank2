"""Simulación completa del RaspTank en el PC.

    ros2 launch rasptank_bringup sim.launch.py                      # mundo de la misión
    ros2 launch rasptank_bringup sim.launch.py world:=linea         # sigue-líneas
    ros2 launch rasptank_bringup sim.launch.py mission:=true autostart:=true
    ros2 launch rasptank_bringup sim.launch.py world:=vacio route:=true

Arranca los mismos nodos que el robot real (base y brazo con driver fake),
el simulador del mundo y de los sensores, el detector y RViz. El código del
alumno se conecta a los mismos topics que en el robot real.
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
    config = lambda name: PathJoinSubstitution([share, 'config', name])  # noqa: E731
    arg = LaunchConfiguration
    # config/sort.yaml with sort:=true, the empty no_sort.yaml otherwise
    sort_config = PathJoinSubstitution([share, 'config', PythonExpression(
        ["'sort.yaml' if '", arg('sort'), "' == 'true' else 'no_sort.yaml'"])])
    cmd_topic = PythonExpression(["'cmd_vel_safe' if '", arg('governor'), "' == 'true' else 'cmd_vel'"])
    world = PathJoinSubstitution([share, 'worlds', PythonExpression(["'", arg('world'), "' + '.yaml'"])])
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value='~/.rasptank/calibracion_sim.yaml',
                              description='calibración de la base simulada (none = sin fichero)'),
        DeclareLaunchArgument('world', default_value='mision',
                              description='mision, linea, vacio o tu mundo en worlds/'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('mission', default_value='false',
                              description='nodo de la misión coger y llevar el cubo'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='arrancar la misión o la ruta sin llamar al servicio'),
        DeclareLaunchArgument('sort', default_value='false',
                              description='clasificar varios cubos (config/sort.yaml)'),
        DeclareLaunchArgument('cube_detector', default_value='color',
                              description='color o yolo (con yolo_model)'),
        DeclareLaunchArgument('yolo_model', default_value='',
                              description='modelo .onnx de tools/yolo/entrenar_yolo.py'),
        DeclareLaunchArgument('governor', default_value='true',
                              description='limitar la velocidad con el ultrasonido'),
        DeclareLaunchArgument('route', default_value='false',
                              description='seguidor de rutas (puntos clicados en RViz)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'rsp.launch.py']))),
        Node(package='rasptank_hw', executable='base_node', name='base_node',
             parameters=[config('robot.yaml'), {'driver': 'fake', 'cmd_topic': cmd_topic,
                                                'calibration_file': arg('calibration_file')}]),
        Node(package='rasptank_hw', executable='motor_watchdog_node', name='motor_watchdog',
             parameters=[config('robot.yaml'), {'driver': 'fake'}]),
        Node(package='rasptank_hw', executable='governor_node', name='governor',
             parameters=[config('robot.yaml')], condition=IfCondition(arg('governor'))),
        Node(package='rasptank_hw', executable='arm_node', name='arm_node',
             parameters=[config('robot.yaml'), {'driver': 'fake'}]),
        Node(package='rasptank_hw', executable='signals_node', name='signals_node',
             parameters=[config('robot.yaml'), {'driver': 'fake'}]),
        Node(package='rasptank_sim', executable='world_node', name='world_node',
             parameters=[{'world_file': world}]),
        Node(package='rasptank_perception', executable='detector_node', name='detector_node',
             parameters=[config('perception.yaml'), sort_config,
                         {'cube_detector': arg('cube_detector'), 'yolo_model': arg('yolo_model')}],
             condition=IfCondition(arg('detector'))),
        Node(package='rasptank_mission', executable='mission_node', name='mission_node',
             parameters=[config('mission.yaml'), sort_config, {'autostart': arg('autostart')}],
             condition=IfCondition(arg('mission'))),
        Node(package='rasptank_mission', executable='route_node', name='route_node',
             parameters=[config('mission.yaml'), {'autostart': arg('autostart')}],
             condition=IfCondition(arg('route'))),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'rasptank.rviz'])],
             condition=IfCondition(arg('rviz'))),
    ])
