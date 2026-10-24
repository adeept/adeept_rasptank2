"""Sensors of the real RaspTank: ultrasonic, line followers and battery.

Publishes:
  /ultrasonic     sensor_msgs/Range               (`max_range` = nothing ahead)
  /line_sensors   rasptank_interfaces/LineSensors
  /battery        sensor_msgs/BatteryState

Only for the Raspberry Pi: the simulator (rasptank_sim/world_node)
publishes the same topics on the PC.
"""
import math
import statistics
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Range

from rasptank_interfaces.msg import LineSensors

from .drivers import Ads7830Battery, GpioLineSensors, GpioUltrasonic


class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors_node')
        p = self.declare_parameter
        self.max_range = p('max_range', 2.0).value
        self.ultrasonic = GpioUltrasonic(p('ultrasonic_trigger', 23).value,
                                         p('ultrasonic_echo', 24).value, self.max_range)
        self.line = GpioLineSensors(p('line_left', 22).value, p('line_middle', 27).value,
                                    p('line_right', 17).value, p('line_active_high', True).value)
        self.battery = None
        if p('battery', True).value:
            try:
                self.battery = Ads7830Battery(p('battery_address', 0x48).value,
                                              p('battery_channel', 0).value)
            except OSError as e:
                self.get_logger().warn(f'no battery ADC: {e}')
        self.v_empty = p('battery_empty', 6.0).value
        self.v_full = p('battery_full', 8.4).value
        self.voltages = deque(maxlen=10)

        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.line_pub = self.create_publisher(LineSensors, 'line_sensors', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_timer(1.0 / p('ultrasonic_rate', 10.0).value, self.read_ultrasonic)
        self.create_timer(1.0 / p('line_rate', 20.0).value, self.read_line)
        self.create_timer(0.5, self.read_battery)
        self.get_logger().info('sensors ready')

    def stamp(self):
        return self.get_clock().now().to_msg()

    def read_ultrasonic(self):
        msg = Range()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(30)
        msg.min_range = 0.02
        msg.max_range = self.max_range
        msg.range = min(self.max_range, self.ultrasonic.read())
        self.range_pub.publish(msg)

    def read_line(self):
        left, middle, right = self.line.read()
        msg = LineSensors(left=left, middle=middle, right=right)
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'line_sensor_link'
        self.line_pub.publish(msg)

    def read_battery(self):
        if self.battery is None:
            return
        try:
            self.voltages.append(self.battery.read())
        except OSError as e:
            self.get_logger().warn(f'battery read failed: {e}', throttle_duration_sec=10.0)
            return
        voltage = statistics.median(self.voltages)
        msg = BatteryState()
        msg.header.stamp = self.stamp()
        msg.voltage = float(voltage)
        msg.percentage = float(max(0.0, min(1.0, (voltage - self.v_empty)
                                            / (self.v_full - self.v_empty))))
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(msg)
        if voltage < self.v_empty:
            self.get_logger().warn(f'battery low: {voltage:.2f} V', throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()
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
