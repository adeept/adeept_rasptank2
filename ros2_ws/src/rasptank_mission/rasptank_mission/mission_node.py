"""Mission: find the cube, pick it up and take it to the goal.

Runs the state machine of rasptank_core.mission at `rate` Hz.

Subscribes:
  /detections       vision_msgs/Detection2DArray  (detector_node)
  /odom             nav_msgs/Odometry
  /arm/state        rasptank_interfaces/ArmState
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
"""
import dataclasses

import rclpy
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

from rasptank_core.geometry import Transform, quat_to_yaw, ray_to_plane_z
from rasptank_core.kinematics import Pose2D
from rasptank_core.mission import ESPERA, FALLO, FIN, Mission, MissionInput, MissionParams, Target
from rasptank_interfaces.msg import ArmState

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

        self.pose = Pose2D()
        self.cube = None
        self.goal = None
        self.arm = ArmState()
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
        self.create_subscription(ArmState, 'arm/state', self.on_arm, latched)
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
        if self.arm.header.stamp.sec == 0 and self.arm.header.stamp.nanosec == 0:
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
        self.arm = msg

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
        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            p = d.results[0].pose.pose.position
            if cls == 'cubo':
                hit = ray_to_plane_z(T.t, T.rotate([p.x, p.y, p.z]), self.cube_height)
                if hit is not None and 0.0 < hit[0] < self.max_range:
                    self.cube = Target(float(hit[0]), float(hit[1]), now)
            elif cls == 'meta':
                x, y, _ = T.apply([p.x, p.y, p.z])
                if 0.0 < x < self.max_range:
                    self.goal = Target(float(x), float(y), now)

    def on_start(self, request, response):
        if self.mission.state not in IDLE:
            response.success, response.message = False, f'already running ({self.mission.state})'
            return response
        self.cube = self.goal = None
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
        inp = MissionInput(self.now(), self.pose, self.cube, self.goal, self.arm.pose,
                           self.arm.moving)
        out = self.mission.step(inp)
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

    def publish_state(self):
        text = self.mission.state + (f': {self.mission.info}' if self.mission.info else '')
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
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
