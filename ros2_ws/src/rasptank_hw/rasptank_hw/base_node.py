"""Tracked base of the RaspTank: /cmd_vel -> motors, and open-loop odometry.

Subscribes:
  /cmd_vel          geometry_msgs/Twist   linear.x (m/s), angular.z (rad/s)
                    (`cmd_topic`: cmd_vel_safe when the governor is in between)
  /emergency_stop   std_msgs/Bool         True stops and latches until False
  /battery          sensor_msgs/BatteryState  only with battery compensation
Publishes:
  /odom             nav_msgs/Odometry     integrated from the commands
  TF odom -> base_footprint
  /base/stopped     std_msgs/Bool         (latched) emergency stop state
  /base/driver      std_msgs/String       (latched) "pca9685" or "fake"
  /base/heartbeat   std_msgs/UInt32       from the control loop, for motor_watchdog
  /base/throttle    std_msgs/Float32MultiArray  [left, right] sent to the motors
Services:
  /base/save_calibration    std_srvs/Trigger  write the calibration file
  /base/reset_calibration   std_srvs/Trigger  back to robot.yaml, delete the file

Safety: the motors stop if no command arrives for `cmd_timeout` seconds,
and the twist is clamped to `max_linear` / `max_angular`. If this node
dies, motor_watchdog_node zeroes the motor channels.

Calibration (rasptank_calibration): `max_track_speed`, `track_separation`
(effective: tracks slip when turning), `min_throttle`, `left_gain`,
`right_gain` and `battery_reference` can be changed while running
(ros2 param set) and saved to `calibration_file`, which is loaded at start
and overrides robot.yaml.

With `driver: fake` nothing is moved; the simulator reads /base/throttle.
"""
import os

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32MultiArray, String, UInt32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from rasptank_core.geometry import yaw_to_quat
from rasptank_core.kinematics import (CommandWatchdog, Odometry, limit_twist, saturate_tracks,
                                      tracks_to_throttle, tracks_to_twist, twist_to_tracks)

from .drivers import FakeMotors, PcaMotors

CALIBRATION_PARAMS = ['max_track_speed', 'track_separation', 'min_throttle', 'left_gain',
                      'right_gain', 'battery_reference']


def battery_factor(reference, voltage, low=0.8, high=1.3):
    """Throttle multiplier that keeps the speed of the calibration voltage."""
    if reference <= 0.0 or voltage is None or voltage <= 0.0:
        return 1.0
    return max(low, min(high, reference / voltage))


class BaseNode(Node):
    def __init__(self, **kwargs):
        super().__init__('base_node', **kwargs)
        p = self.declare_parameter
        self.driver_name = p('driver', 'fake').value
        address = p('i2c_address', 0x5f).value
        frequency = p('pwm_frequency', 50).value
        left_channels = list(p('left_channels', [15, 14]).value)
        right_channels = list(p('right_channels', [12, 13]).value)
        self.left_sign = p('left_sign', 1.0).value
        self.right_sign = p('right_sign', 1.0).value
        p('track_separation', 0.12)
        p('max_track_speed', 0.25)
        p('min_throttle', 0.0)
        p('left_gain', 1.0)
        p('right_gain', 1.0)
        p('battery_reference', 0.0)         # V at calibration; 0 = no compensation
        self.max_linear = p('max_linear', 0.2).value
        self.max_angular = p('max_angular', 1.5).value
        self.watchdog = CommandWatchdog(p('cmd_timeout', 0.5).value)
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value
        cmd_topic = p('cmd_topic', 'cmd_vel').value
        calibration_file = p('calibration_file', '~/.rasptank/calibracion.yaml').value
        if calibration_file.strip().lower() in ('', 'none'):
            calibration_file = ''             # no file: nothing loaded or saved
        self.calibration_file = os.path.expanduser(calibration_file) if calibration_file else ''
        rate = p('rate', 50.0).value

        if self.driver_name == 'pca9685':
            self.motors = PcaMotors(address, frequency, left_channels, right_channels)
        elif self.driver_name == 'fake':
            self.motors = FakeMotors()
        else:
            raise ValueError(f'unknown driver {self.driver_name!r} (pca9685 or fake)')
        self.motors.stop()        # whatever a previous (killed) process left running

        self.defaults = {n: self.get_parameter(n).value for n in CALIBRATION_PARAMS}
        self.add_on_set_parameters_callback(self.on_params)
        self.load_calibration()

        self.odom = Odometry()
        self.cmd = (0.0, 0.0)
        self.stopped = False
        self.voltage = None
        self.beats = 0
        self.last_time = self.now()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stopped_pub = self.create_publisher(Bool, 'base/stopped', latched)
        self.driver_pub = self.create_publisher(String, 'base/driver', latched)
        self.heartbeat_pub = self.create_publisher(UInt32, 'base/heartbeat', 10)
        self.throttle_pub = self.create_publisher(Float32MultiArray, 'base/throttle', 10)
        self.odom_pub = self.create_publisher(OdometryMsg, 'odom', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_stop, latched)
        self.create_subscription(BatteryState, 'battery', self.on_battery, 10)
        self.create_service(Trigger, 'base/save_calibration', self.on_save)
        self.create_service(Trigger, 'base/reset_calibration', self.on_reset)
        self.create_timer(1.0 / rate, self.loop)
        self.stopped_pub.publish(Bool(data=False))
        self.driver_pub.publish(String(data=self.driver_name))
        self.get_logger().info(f'base ready (driver {self.driver_name}, commands on /{cmd_topic})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def param(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------ calibration

    def on_params(self, params):
        for prm in params:
            if prm.name in ('max_track_speed', 'track_separation') and prm.value <= 0.0:
                return SetParametersResult(successful=False, reason=f'{prm.name} must be > 0')
            if prm.name == 'min_throttle' and not 0.0 <= prm.value < 1.0:
                return SetParametersResult(successful=False, reason='min_throttle in [0, 1)')
            if prm.name in ('left_gain', 'right_gain') and not 0.5 <= prm.value <= 2.0:
                return SetParametersResult(successful=False, reason=f'{prm.name} in [0.5, 2]')
        return SetParametersResult(successful=True)

    def load_calibration(self):
        if not self.calibration_file or not os.path.exists(self.calibration_file):
            return
        try:
            with open(self.calibration_file) as f:
                data = yaml.safe_load(f) or {}
            values = [Parameter(n, value=float(data[n])) for n in CALIBRATION_PARAMS if n in data]
            for result, prm in zip(self.set_parameters(values), values):
                if not result.successful:
                    self.get_logger().error(f'calibration {prm.name}: {result.reason}')
            self.get_logger().info(f'calibration loaded from {self.calibration_file}')
        except (OSError, ValueError, TypeError, yaml.YAMLError) as e:
            self.get_logger().error(f'cannot read {self.calibration_file}: {e}')

    def on_save(self, request, response):
        if not self.calibration_file:
            response.success, response.message = False, 'calibration_file is empty'
            return response
        data = {n: float(self.param(n)) for n in CALIBRATION_PARAMS}
        try:
            os.makedirs(os.path.dirname(self.calibration_file) or '.', exist_ok=True)
            with open(self.calibration_file, 'w') as f:
                f.write('# RaspTank odometry calibration (rasptank_calibration)\n')
                yaml.safe_dump(data, f, default_flow_style=False)
        except OSError as e:
            response.success, response.message = False, str(e)
            return response
        response.success, response.message = True, f'saved to {self.calibration_file}'
        self.get_logger().info(response.message)
        return response

    def on_reset(self, request, response):
        self.set_parameters([Parameter(n, value=v) for n, v in self.defaults.items()])
        if self.calibration_file and os.path.exists(self.calibration_file):
            os.remove(self.calibration_file)
        response.success, response.message = True, 'calibration reset to robot.yaml'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------ inputs

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

    def on_battery(self, msg):
        self.voltage = msg.voltage

    # ------------------------------------------------------------ loop

    def loop(self):
        now = self.now()
        dt = max(0.0, min(0.2, now - self.last_time))
        self.last_time = now
        v, w = self.cmd
        if self.stopped or self.watchdog.expired(now):
            v, w = 0.0, 0.0
        v, w = limit_twist(v, w, self.max_linear, self.max_angular)
        separation = self.param('track_separation')
        max_speed = self.param('max_track_speed')
        left, right = saturate_tracks(*twist_to_tracks(v, w, separation), max_speed)
        # gains act on the track speed, before the dead band compensation
        k = battery_factor(self.param('battery_reference'), self.voltage)
        t_left, t_right = tracks_to_throttle(left * self.param('left_gain') * k,
                                             right * self.param('right_gain') * k,
                                             max_speed, self.param('min_throttle'))
        self.motors.set(self.left_sign * t_left, self.right_sign * t_right)
        self.throttle_pub.publish(Float32MultiArray(data=[float(t_left), float(t_right)]))
        self.beats += 1
        self.heartbeat_pub.publish(UInt32(data=self.beats % 2**32))
        # odometry from what the tracks were asked for (after saturation): the
        # gains exist so that the motors really do it
        v_out, w_out = tracks_to_twist(left, right, separation)
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a real error; otherwise a callback was cut by the shutdown
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
