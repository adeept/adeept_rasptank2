"""Independent motor watchdog: stops the tracks if base_node dies or hangs.

The PCA9685 keeps generating the last PWM it was given, so if base_node is
killed (kill -9, out of memory) or its loop hangs, the robot would keep
driving. This separate process watches /base/heartbeat, which base_node
publishes from its control loop, and when it stops for `timeout` seconds
it writes "full off" straight into the motor channels of the PCA9685.
It does that again every `repeat` seconds while the heartbeat is missing
(harmless: the motors are off anyway) and also while the emergency stop
is active.

It cannot protect against its own death or a dead I2C bus: that is what
the respawn in the launch file and, on the floor, the battery switch are for.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, UInt32

from rasptank_core.kinematics import CommandWatchdog

from .drivers import FakeMotorKiller, PcaMotorKiller


class MotorWatchdogNode(Node):
    def __init__(self, **kwargs):
        super().__init__('motor_watchdog', **kwargs)
        p = self.declare_parameter
        driver = p('driver', 'fake').value
        channels = list(p('left_channels', [15, 14]).value) + list(p('right_channels', [12, 13]).value)
        self.heartbeat = CommandWatchdog(p('timeout', 0.3).value)
        self.repeat = p('repeat', 0.5).value
        if driver == 'pca9685':
            self.killer = PcaMotorKiller(p('i2c_address', 0x5f).value, channels)
        elif driver == 'fake':
            self.killer = FakeMotorKiller()
        else:
            raise ValueError(f'unknown driver {driver!r} (pca9685 or fake)')
        self.stopped = False
        self.tripped = False
        self.last_zero = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt32, 'base/heartbeat', self.on_beat, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_stop, latched)
        self.create_timer(0.05, self.check)
        self.get_logger().info(f'motor watchdog ready (driver {driver}, channels {channels})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_beat(self, msg):
        self.heartbeat.feed(self.now())
        if self.tripped:
            self.tripped = False
            self.get_logger().info('base_node heartbeat back')

    def on_stop(self, msg):
        self.stopped = msg.data

    def check(self):
        now = self.now()
        missing = self.heartbeat.expired(now)
        if not (missing or self.stopped):
            return
        if missing and not self.tripped and self.heartbeat.last is not None:
            self.tripped = True
            self.get_logger().error('no heartbeat from base_node: motors switched off')
        if self.last_zero is None or now - self.last_zero >= self.repeat:
            self.last_zero = now
            try:
                self.killer.zero()
            except OSError as e:
                self.get_logger().error(f'cannot switch the motors off: {e}',
                                        throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorWatchdogNode()
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
