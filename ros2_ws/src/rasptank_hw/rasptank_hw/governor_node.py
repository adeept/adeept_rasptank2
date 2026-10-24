"""Speed governor node (rasptank_core.governor) between /cmd_vel and the base.

Subscribes /cmd_vel and /ultrasonic, publishes /cmd_vel_safe (base_node
listens there when the governor is on: `cmd_topic: cmd_vel_safe`) and
/governor/active (std_msgs/Bool, True while it is limiting).

It runs on the robot, next to base_node, so it does not depend on the WiFi.
"""
import dataclasses

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from rasptank_core.governor import GovernorParams, govern


class GovernorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('governor', **kwargs)
        values = {f.name: self.declare_parameter(f.name, f.default).value
                  for f in dataclasses.fields(GovernorParams)}
        self.p = GovernorParams(**values)
        self.distance = None
        self.stamp = None
        self.active = None
        self.pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.active_pub = self.create_publisher(Bool, 'governor/active', 10)
        self.create_subscription(Range, 'ultrasonic', self.on_range, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.get_logger().info(f'governor ready: stop at {self.p.stop_distance:.2f} m, '
                               f'slow from {self.p.slow_distance:.2f} m')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_range(self, msg):
        self.distance = msg.range
        self.stamp = self.now()

    def on_cmd(self, msg):
        age = None if self.stamp is None else self.now() - self.stamp
        v, w, limited = govern(msg.linear.x, msg.angular.z, self.distance, age, self.p)
        out = Twist()
        out.linear.x, out.angular.z = float(v), float(w)
        self.pub.publish(out)
        if limited != self.active:
            self.active = limited
            self.active_pub.publish(Bool(data=limited))


def main(args=None):
    rclpy.init(args=args)
    node = GovernorNode()
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
