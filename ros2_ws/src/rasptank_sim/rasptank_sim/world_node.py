"""Light simulator of the RaspTank: the world and the sensors.

The robot itself is the same as on the real one: base_node and arm_node
with `driver: fake` publish the odometry (here the true pose) and the
joint states, and robot_state_publisher the TF of the URDF. This node
reads that TF and publishes what the sensors would see:

  /camera/image/compressed  /camera/camera_info  synthetic camera
  /ultrasonic               sensor_msgs/Range
  /line_sensors             rasptank_interfaces/LineSensors
  /battery                  sensor_msgs/BatteryState (constant)
  /sim/markers              visualization_msgs/MarkerArray (RViz)
  /sim/cube_pose            geometry_msgs/PoseStamped
  /sim/cube_in_goal         std_msgs/Bool
  /sim/collision            std_msgs/Bool

The gripper picks the cube up when it closes around it and drops it when
it opens. There is no physics: the robot goes through walls, but the
collision is reported.
"""
import math

import cv2
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, JointState, Range
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rasptank_core.geometry import Transform, yaw_to_quat
from rasptank_core.vision import intrinsics_from_hfov
from rasptank_interfaces.msg import LineSensors

from .render import Renderer
from .world import World


def tf_to_transform(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return Transform.from_quat([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


class WorldNode(Node):
    def __init__(self):
        super().__init__('world_node')
        p = self.declare_parameter
        world_file = p('world_file', '').value
        self.fixed_frame = p('fixed_frame', 'odom').value
        self.width = p('camera_width', 640).value
        self.height = p('camera_height', 480).value
        hfov = p('camera_hfov', 62.2).value
        self.image_noise = p('image_noise', 2.0).value
        self.ultrasonic_noise = p('ultrasonic_noise', 0.005).value
        self.line_spacing = p('line_sensor_spacing', 0.015).value
        self.gripper_closed = p('gripper_closed', 0.15).value
        self.gripper_open = p('gripper_open', 0.3).value
        self.grasp_radius = p('grasp_radius', 0.03).value
        self.max_range = 2.0

        if not world_file:
            raise ValueError('world_file parameter is required')
        with open(world_file) as f:
            self.world = World(yaml.safe_load(f))
        self.renderer = Renderer(self.world)
        self.K = intrinsics_from_hfov(self.width, self.height, hfov)
        self.gripper = None
        self.collision = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', 2)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 2)
        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.line_pub = self.create_publisher(LineSensors, 'line_sensors', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'sim/markers', 2)
        self.cube_pub = self.create_publisher(PoseStamped, 'sim/cube_pose', 10)
        self.in_goal_pub = self.create_publisher(Bool, 'sim/cube_in_goal', 10)
        self.collision_pub = self.create_publisher(Bool, 'sim/collision', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)

        self.create_timer(1.0 / p('camera_rate', 10.0).value, self.publish_camera)
        self.create_timer(0.1, self.publish_ultrasonic)
        self.create_timer(0.05, self.publish_line)
        self.create_timer(0.05, self.update_cube)
        self.create_timer(0.2, self.publish_markers)
        self.create_timer(1.0, self.publish_battery)
        self.get_logger().info(f'world {world_file} loaded')

    # ------------------------------------------------------------ helpers

    def lookup(self, frame):
        try:
            return tf_to_transform(self.tf_buffer.lookup_transform(self.fixed_frame, frame, Time()))
        except TransformException:
            return None

    def stamp(self):
        return self.get_clock().now().to_msg()

    # ------------------------------------------------------------ gripper and cube

    def on_joints(self, msg):
        if 'gripper_joint' not in msg.name:
            return
        angle = msg.position[msg.name.index('gripper_joint')]
        previous = self.gripper
        self.gripper = angle
        if previous is None:
            return
        if previous > self.gripper_closed >= angle:
            T = self.lookup('gripper_center_link')
            if T is not None and self.world.try_grasp(T.t, self.grasp_radius):
                self.get_logger().info('cube grasped')
        elif previous < self.gripper_open <= angle:
            if self.world.release():
                self.get_logger().info('cube released'
                                       + (' inside the goal' if self.world.cube_in_goal() else ''))

    def update_cube(self):
        cube = self.world.cube
        if cube is not None and cube.held:
            T = self.lookup('gripper_center_link')
            if T is not None:
                self.world.carry(T.t)
        if cube is not None:
            msg = PoseStamped()
            msg.header.stamp = self.stamp()
            msg.header.frame_id = self.fixed_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
                (float(v) for v in cube.position)
            q = yaw_to_quat(cube.yaw)
            msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
            self.cube_pub.publish(msg)
            self.in_goal_pub.publish(Bool(data=self.world.cube_in_goal()))
        base = self.lookup('base_footprint')
        if base is not None:
            yaw = math.atan2(base.R[1, 0], base.R[0, 0])
            hit = self.world.collides(base.t[0], base.t[1], yaw)
            if hit and not self.collision:
                self.get_logger().warn('collision: the robot is touching a wall or obstacle')
            self.collision = hit
            self.collision_pub.publish(Bool(data=hit))

    # ------------------------------------------------------------ sensors

    def publish_camera(self):
        T = self.lookup('camera_optical_frame')
        if T is None:
            return
        img = self.renderer.render(T, self.K, self.width, self.height, self.image_noise)
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg = CompressedImage(format='jpeg', data=jpeg.tobytes())
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(msg)
        info = CameraInfo(width=self.width, height=self.height, distortion_model='plumb_bob')
        info.header = msg.header
        info.k = self.K.flatten().tolist()
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.K[0, 0], 0.0, self.K[0, 2], 0.0, 0.0, self.K[1, 1], self.K[1, 2], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def publish_ultrasonic(self):
        T = self.lookup('ultrasonic_link')
        if T is None:
            return
        yaw = math.atan2(T.R[1, 0], T.R[0, 0])
        msg = Range()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(30)
        msg.min_range = 0.02
        msg.max_range = self.max_range
        msg.range = float(self.world.ultrasonic(T.t[0], T.t[1], yaw, T.t[2], self.max_range,
                                                msg.field_of_view,
                                                noise=self.ultrasonic_noise))
        self.range_pub.publish(msg)

    def publish_line(self):
        T = self.lookup('line_sensor_link')
        if T is None:
            return
        values = []
        for offset in (self.line_spacing, 0.0, -self.line_spacing):   # left, middle, right
            x, y, _ = T.apply([0.0, offset, 0.0])
            values.append(self.world.on_line(x, y))
        msg = LineSensors(left=values[0], middle=values[1], right=values[2])
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'line_sensor_link'
        self.line_pub.publish(msg)

    def publish_battery(self):
        msg = BatteryState(voltage=8.0, percentage=0.83, present=True)
        msg.header.stamp = self.stamp()
        self.battery_pub.publish(msg)

    # ------------------------------------------------------------ RViz

    def _marker(self, ns, mid, mtype, color, scale=(1.0, 1.0, 1.0)):
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = self.stamp()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = (float(s) for s in scale)
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3] if len(color) > 3 else 1.0)
        m.pose.orientation.w = 1.0
        return m

    def _box(self, ns, mid, x, y, z, yaw, size, color):
        m = self._marker(ns, mid, Marker.CUBE, color, size)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        q = yaw_to_quat(yaw)
        m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
        return m

    def publish_markers(self):
        w = self.world
        out = MarkerArray()
        cx, cy = w.arena_center
        sx, sy = w.arena_size
        t = 0.02
        for i, (x, y, lx, ly) in enumerate([(cx + sx / 2, cy, t, sy), (cx - sx / 2, cy, t, sy),
                                            (cx, cy + sy / 2, sx, t), (cx, cy - sy / 2, sx, t)]):
            out.markers.append(self._box('walls', i, x, y, w.wall_height / 2, 0.0,
                                         (lx, ly, w.wall_height), (0.75, 0.75, 0.8, 0.6)))
        for i, o in enumerate(w.obstacles):
            out.markers.append(self._box('obstacles', i, o.x, o.y, o.height / 2, o.yaw,
                                         (o.size[0], o.size[1], o.height),
                                         (o.color[2] / 255, o.color[1] / 255, o.color[0] / 255)))
        if w.line:
            m = self._marker('line', 0, Marker.LINE_STRIP, (0.05, 0.05, 0.05),
                             (w.line_width, 0, 0))
            for x, y in w.line:
                pt = m.pose.position.__class__(x=float(x), y=float(y), z=0.001)
                m.points.append(pt)
            out.markers.append(m)
        if w.goal is not None:
            g = w.goal
            out.markers.append(self._box('goal', 0, g.x, g.y, g.board[1] / 2, g.yaw,
                                         (0.01, g.board[0], g.board[1]), (0.95, 0.95, 0.95)))
            ax, ay = g.area
            out.markers.append(self._box('goal', 1, ax, ay, 0.001, 0.0,
                                         (2 * g.area_radius, 2 * g.area_radius, 0.002),
                                         (0.3, 0.8, 0.3, 0.5)))
            out.markers[-1].type = Marker.CYLINDER
            label = self._marker('goal', 2, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), (0, 0, 0.05))
            label.pose.position.x, label.pose.position.y = g.x, g.y
            label.pose.position.z = g.board[1] + 0.05
            label.text = f'META (ArUco {g.marker_id})'
            out.markers.append(label)
        if w.cube is not None:
            c = w.cube
            col = (c.color[2] / 255, c.color[1] / 255, c.color[0] / 255)
            out.markers.append(self._box('cube', 0, *c.position, c.yaw, (c.size,) * 3, col))
        self.marker_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
