"""Tracked base of the RaspTank: /cmd_vel -> motors, and open-loop odometry.

Subscribes:
  /cmd_vel          geometry_msgs/Twist   linear.x (m/s), angular.z (rad/s)
  /emergency_stop   std_msgs/Bool         True stops and latches until False
Publishes:
  /odom             nav_msgs/Odometry     integrated from the commands
  TF odom -> base_footprint
  /base/stopped     std_msgs/Bool         (latched) emergency stop state

Safety: the motors stop if no /cmd_vel arrives for `cmd_timeout` seconds,
and the twist is clamped to `max_linear` / `max_angular`.

With `driver: fake` nothing is moved: the odometry is then the robot pose
of the simulator.
"""
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from rasptank_core.geometry import yaw_to_quat
from rasptank_core.kinematics import (CommandWatchdog, Odometry, limit_twist, saturate_tracks,
                                      tracks_to_throttle, tracks_to_twist, twist_to_tracks)

from .drivers import FakeMotors, PcaMotors


class BaseNode(Node):
    def __init__(self):
        super().__init__('base_node')
        p = self.declare_parameter
        self.driver_name = p('driver', 'fake').value
        address = p('i2c_address', 0x5f).value
        frequency = p('pwm_frequency', 50).value
        left_channels = list(p('left_channels', [15, 14]).value)
        right_channels = list(p('right_channels', [12, 13]).value)
        self.left_sign = p('left_sign', 1.0).value
        self.right_sign = p('right_sign', 1.0).value
        self.separation = p('track_separation', 0.12).value
        self.max_track_speed = p('max_track_speed', 0.25).value
        self.min_throttle = p('min_throttle', 0.0).value
        self.max_linear = p('max_linear', 0.2).value
        self.max_angular = p('max_angular', 1.5).value
        self.watchdog = CommandWatchdog(p('cmd_timeout', 0.5).value)
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value
        rate = p('rate', 50.0).value

        if self.driver_name == 'pca9685':
            self.motors = PcaMotors(address, frequency, left_channels, right_channels)
        elif self.driver_name == 'fake':
            self.motors = FakeMotors()
        else:
            raise ValueError(f'unknown driver {self.driver_name!r} (pca9685 or fake)')

        self.odom = Odometry()
        self.cmd = (0.0, 0.0)
        self.stopped = False
        self.last_time = self.now()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stopped_pub = self.create_publisher(Bool, 'base/stopped', latched)
        self.odom_pub = self.create_publisher(OdometryMsg, 'odom', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_stop, latched)
        self.create_timer(1.0 / rate, self.loop)
        self.stopped_pub.publish(Bool(data=False))
        self.get_logger().info(f'base ready (driver {self.driver_name})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.watchdog.feed(self.now())

    def on_stop(self, msg):
        if msg.data != self.stopped:
            self.stopped = msg.data
            self.get_logger().warn('EMERGENCY STOP' if msg.data else 'emergency stop released')
            self.stopped_pub.publish(Bool(data=self.stopped))
        if self.stopped:
            self.motors.stop()

    def loop(self):
        now = self.now()
        dt = max(0.0, min(0.2, now - self.last_time))
        self.last_time = now
        v, w = self.cmd
        if self.stopped or self.watchdog.expired(now):
            v, w = 0.0, 0.0
        v, w = limit_twist(v, w, self.max_linear, self.max_angular)
        left, right = saturate_tracks(*twist_to_tracks(v, w, self.separation),
                                      self.max_track_speed)
        t_left, t_right = tracks_to_throttle(left, right, self.max_track_speed, self.min_throttle)
        self.motors.set(self.left_sign * t_left, self.right_sign * t_right)
        # odometry from what the tracks were really asked for (after saturation)
        v_out, w_out = tracks_to_twist(left, right, self.separation)
        pose = self.odom.update(v_out, w_out, dt)
        self.publish_odom(pose, v_out, w_out)

    def publish_odom(self, pose, v, w):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(pose.yaw)
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.odom_pub.publish(msg)
        if self.tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = pose.x
            t.transform.translation.y = pose.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)

    def destroy_node(self):
        self.motors.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
