"""ROS 2 connection of the app, to one domain at a time.

Its own rclpy context and spinning thread, so the domain can be changed
while the app runs. Keeps a Snapshot of the robot, and drives it: the
emergency stop, short teleoperation, arm poses, camera tilt. The
calibration uses this same node (rasptank_calibration.Session).
"""
import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, JointState, Range
from std_msgs.msg import Bool, String, UInt32

from rasptank_core.geometry import quat_to_yaw
from rasptank_interfaces.msg import ArmState, LineSensors

from .status import Snapshot

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _pose(msg):
    p = msg.pose.pose
    o = p.orientation
    return p.position.x, p.position.y, math.degrees(quat_to_yaw(o.x, o.y, o.z, o.w))


class RosLink:
    def __init__(self, domain):
        self.domain = domain
        self.lock = threading.Lock()
        self.snap = Snapshot()
        self.jpeg = None
        self.frames = deque(maxlen=20)
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = n = Node(f'rasptank_app_{domain}', context=self.context)
        sub = n.create_subscription
        sub(String, 'base/driver', lambda m: self._set(driver=m.data), LATCHED)
        sub(Bool, 'base/stopped', lambda m: self._set(stopped=m.data), LATCHED)
        sub(UInt32, 'base/heartbeat', lambda m: self._set(heartbeat_at=time.monotonic()), 10)
        sub(BatteryState, 'battery', lambda m: self._set(voltage=m.voltage,
                                                         battery_pct=m.percentage), 10)
        sub(Range, 'ultrasonic', lambda m: self._set(distance=m.range), qos_profile_sensor_data)
        sub(LineSensors, 'line_sensors', lambda m: self._set(line=(m.left, m.middle, m.right)), 10)
        sub(Odometry, 'odom', lambda m: self._set(pose=_pose(m)), 10)
        sub(Odometry, 'sim/ground_truth', lambda m: self._set(truth=_pose(m)), 10)
        sub(String, 'mission/state', lambda m: self._set(mission=m.data), LATCHED)
        sub(ArmState, 'arm/state', lambda m: self._set(arm=m.pose + (' (moviendo)' if m.moving
                                                                     else '')), LATCHED)
        sub(Bool, 'governor/active', lambda m: self._set(governor=m.data), 10)
        sub(CompressedImage, 'camera/image/compressed', self._on_image, qos_profile_sensor_data)
        self.stop_pub = n.create_publisher(Bool, 'emergency_stop', LATCHED)
        self.cmd_pub = n.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = n.create_publisher(String, 'arm/pose', 10)
        self.joint_pub = n.create_publisher(JointState, 'arm/joint_command', 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self._session = None

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:      # context shut down while closing
            pass

    def close(self):
        if self._session is not None:
            self._session.close()
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)

    # ------------------------------------------------------------ state

    def _set(self, **values):
        with self.lock:
            for k, v in values.items():
                setattr(self.snap, k, v)

    def _on_image(self, msg):
        now = time.monotonic()
        with self.lock:
            self.jpeg = bytes(msg.data)
            self.frames.append(now)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            s = Snapshot(**{k: getattr(self.snap, k) for k in self.snap.__dataclass_fields__})
            frames = [t for t in self.frames if now - t < 2.0]
        s.now = now
        s.camera_hz = len(frames) / 2.0
        return s

    def last_jpeg(self):
        with self.lock:
            return self.jpeg

    # ------------------------------------------------------------ commands

    def emergency_stop(self, on):
        self.stop_pub.publish(Bool(data=bool(on)))

    def drive(self, v, w):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    def arm_pose(self, name):
        self.pose_pub.publish(String(data=name))

    def camera_tilt(self, radians):
        self.joint_pub.publish(JointState(name=['camera_tilt_joint'], position=[float(radians)]))

    def session(self):
        """Calibration session on this node (created on first use)."""
        if self._session is None:
            from rasptank_calibration.session import Session
            self._session = Session(node=self.node)
        return self._session
