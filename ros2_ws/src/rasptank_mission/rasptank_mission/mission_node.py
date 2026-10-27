"""Mission: find the cube, pick it up and take it to the goal.

Runs the state machine of rasptank_core.mission at `rate` Hz.

Subscribes:
  /detections       vision_msgs/Detection2DArray  (detector_node)
  /odom             nav_msgs/Odometry
  /arm/state        std_msgs/String (JSON, rasptank_core.arm.ArmStatus)
  /emergency_stop   std_msgs/Bool                 True aborts the mission
Publishes:
  /cmd_vel          geometry_msgs/Twist  (only while the mission runs)
  /arm/pose         std_msgs/String
  /mission/state    std_msgs/String (latched) "STATE: info"
  /mission/markers  visualization_msgs/MarkerArray (state, cube, goal)
Services:
  /mission/start    std_srvs/Trigger
  /mission/stop     std_srvs/Trigger

The detections are turned into positions in base_footprint with TF. The
cube's distance from its apparent size is rough, so its position is the
point where the ray through the box centre meets the plane at the
height of the cube centre (`cube_height`).

`mode: sort` sorts several cubes: each class of `sort_classes` goes to the
ArUco marker of the same position in `sort_goals` (rasptank_core.sorting
chooses the cube and the goal). The pick-and-place cycle repeats until
`max_objects` are delivered (0: until no cube is left to find).
"""
import dataclasses

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from rasptank_core.arm import ArmStatus
from rasptank_core.geometry import Transform, quat_to_yaw, ray_to_plane_z
from rasptank_core.kinematics import Pose2D
from rasptank_core.mission import (APROXIMAR_CUBO, BUSCAR_CUBO, ESPERA, FALLO, FIN, Mission,
                                   MissionInput, MissionParams, Target)
from rasptank_core.sorting import Sorter

IDLE = (ESPERA, FIN, FALLO)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        values = {}
        for f in dataclasses.fields(MissionParams):
            values[f.name] = self.declare_parameter(f.name, f.default).value
        self.mission = Mission(MissionParams(**values))
        self.cube_height = self.declare_parameter('cube_height', 0.02).value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.max_range = self.declare_parameter('max_target_range', 2.5).value
        rate = self.declare_parameter('rate', 20.0).value
        self.sort = self.declare_parameter('mode', 'single').value == 'sort'
        classes = list(self.declare_parameter('sort_classes', ['cubo_rojo', 'cubo_azul']).value)
        goals = [int(g) for g in self.declare_parameter('sort_goals', [0, 1]).value]
        if self.sort and len(classes) != len(goals):
            raise ValueError('sort_classes and sort_goals need the same length')
        self.sort_goals = dict(zip(classes, goals))
        self.max_objects = self.declare_parameter('max_objects', 0).value
        self.sorter = Sorter(self.sort_goals)
        self.final = ''
        self.previous_state = None

        self.pose = Pose2D()
        self.cube = None
        self.goal = None
        self.arm = ArmStatus()
        self.sent_pose = None
        self.last_state = None
        self.was_running = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(String, 'arm/pose', 10)
        self.state_pub = self.create_publisher(String, 'mission/state', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'mission/markers', 2)
        self.create_subscription(Detection2DArray, 'detections', self.on_detections, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(String, 'arm/state', self.on_arm, latched)
        self.create_subscription(Bool, 'emergency_stop', self.on_emergency, latched)
        self.create_service(Trigger, 'mission/start', self.on_start)
        self.create_service(Trigger, 'mission/stop', self.on_stop)
        self.create_timer(1.0 / rate, self.loop)
        self._auto_timer = None
        if self.declare_parameter('autostart', False).value:
            self._auto_timer = self.create_timer(1.0, self._autostart)
        self.publish_state()
        self.get_logger().info('mission ready: ros2 service call /mission/start std_srvs/srv/Trigger')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _autostart(self):
        # wait until the arm node is up, so the first pose is not lost
        if not self.arm.received:
            return
        self._auto_timer.cancel()
        self.mission.start()
        self.get_logger().info('mission started (autostart)')

    # ------------------------------------------------------------ inputs

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                           quat_to_yaw(q.x, q.y, q.z, q.w))

    def on_arm(self, msg):
        self.arm = ArmStatus.from_json(msg.data)

    def on_emergency(self, msg):
        if msg.data and self.mission.state not in IDLE:
            self.mission.stop()
            self.get_logger().warn('emergency stop: mission aborted')

    def on_detections(self, msg):
        if not msg.detections:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'no TF to the camera: {e}', throttle_duration_sec=5.0)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        T = Transform.from_quat([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        now = self.now()   # reception time: the Pi and PC clocks may differ
        cube_classes = self.sort_goals if self.sort else ('cubo',)
        seen = []          # (class, x, y) in the robot frame
        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            p = d.results[0].pose.pose.position
            if cls in cube_classes:
                hit = ray_to_plane_z(T.t, T.rotate([p.x, p.y, p.z]), self.cube_height)
                if hit is not None and 0.0 < hit[0] < self.max_range:
                    seen.append((cls, float(hit[0]), float(hit[1])))
            elif cls.startswith('meta'):
                x, y, _ = T.apply([p.x, p.y, p.z])
                if 0.0 < x < self.max_range:
                    seen.append((cls, float(x), float(y)))
        if self.sort:
            self.sorter.observe(seen, self.pose, now)
            return
        cubes = [s for s in seen if s[0] == 'cubo']
        if cubes:
            _, x, y = min(cubes, key=lambda c: c[1] ** 2 + c[2] ** 2)     # the nearest
            self.cube = Target(x, y, now)
        goals = [s for s in seen if s[0] == 'meta']
        if goals:
            self.goal = Target(goals[0][1], goals[0][2], now)

    def on_start(self, request, response):
        if self.mission.state not in IDLE:
            response.success, response.message = False, f'already running ({self.mission.state})'
            return response
        self.cube = self.goal = None
        self.sorter = Sorter(self.sort_goals)
        self.final = ''
        self.mission.start()
        response.success, response.message = True, 'mission started'
        return response

    def on_stop(self, request, response):
        self.mission.stop()
        self.cmd_pub.publish(Twist())
        response.success, response.message = True, 'mission stopped'
        return response

    # ------------------------------------------------------------ loop

    def loop(self):
        now = self.now()
        cube, goal = self.cube, self.goal
        if self.sort:
            state = self.mission.state
            if state == APROXIMAR_CUBO and self.previous_state != APROXIMAR_CUBO:
                self.sorter.lock()
                self.get_logger().info(f'going for a {self.sorter.locked_class}')
            elif state == BUSCAR_CUBO:
                self.sorter.unlock()
            self.previous_state = state
            cube = self.sorter.cube_target(self.pose, now)
            goal = self.sorter.goal_target(self.pose, now)
        inp = MissionInput(now, self.pose, cube, goal, self.arm.pose, self.arm.moving)
        out = self.mission.step(inp)
        if self.sort:
            out = self.next_object(out)
        running = out.state not in IDLE
        if running:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(out.v), float(out.w)
            self.cmd_pub.publish(cmd)
        elif self.was_running:
            self.cmd_pub.publish(Twist())
        self.was_running = running
        if running and out.arm_pose and out.arm_pose != self.sent_pose:
            self.pose_pub.publish(String(data=out.arm_pose))
            self.sent_pose = out.arm_pose
        if not running:
            self.sent_pose = None
        if (out.state, out.info) != self.last_state:
            if out.state != (self.last_state or ('',))[0]:
                text = f'{out.state} {out.info}'.strip()
                if out.state == FALLO:
                    self.get_logger().error(text)
                else:
                    self.get_logger().info(text)
            self.last_state = (out.state, out.info)
            self.publish_state()
        self.publish_markers(out)

    def next_object(self, out):
        """Sorting: after each delivery start again, until there is nothing left."""
        done = len(self.sorter.delivered)
        if out.state == FIN and not self.final:
            self.sorter.deliver()
            done += 1
            self.get_logger().info(f'{self.sorter.delivered[-1]} delivered ({done})')
            if self.max_objects and done >= self.max_objects:
                self.final = f'FIN: {done} cubos clasificados'
                self.publish_state()
            else:
                self.mission.start()
        elif out.state == FALLO and done and 'cubo' in self.mission.info and not self.final:
            self.final = f'FIN: {done} cubos clasificados (no quedan más)'
            self.publish_state()
        out.state = self.mission.state
        return out

    def publish_state(self):
        if self.final:
            self.state_pub.publish(String(data=self.final))
            return
        text = self.mission.state + (f': {self.mission.info}' if self.mission.info else '')
        if self.sort:
            text += f' [{len(self.sorter.delivered)} clasificados]'
        self.state_pub.publish(String(data=text))

    def publish_markers(self, out):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        text = Marker(ns='mission', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.header.frame_id = self.base_frame
        text.header.stamp = stamp
        text.pose.position.z = 0.3
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color = ColorRGBA(r=1.0, g=1.0, b=0.3, a=1.0)
        text.text = out.state + (f'\n{out.info}' if out.info else '')
        markers.markers.append(text)
        for mid, target, color in ((1, self.cube, (1.0, 0.2, 0.2)), (2, self.goal, (0.2, 1.0, 0.2))):
            m = Marker(ns='mission', id=mid, type=Marker.SPHERE)
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp
            fresh = target is not None and self.now() - target.stamp < 1.0
            m.action = Marker.ADD if fresh else Marker.DELETE
            if fresh:
                m.pose.position.x, m.pose.position.y = target.x, target.y
                m.pose.position.z = 0.02
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.05
                m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a real error; otherwise a callback was cut by the shutdown
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
