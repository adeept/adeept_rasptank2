"""LEDs and buzzer of the RaspTank.

Subscribes:
  /leds             std_msgs/ColorRGBA  colour of all the LEDs (r, g, b 0-1, a = brightness)
  /buzzer           std_msgs/String     a tune: "C5:0.2 E5:0.2 -:0.1 G5:0.4" (note:seconds,
                                        "-" is a rest); "" stops it
  /emergency_stop   std_msgs/Bool       True: LEDs red and the buzzer stops
Publishes:
  /leds/state       std_msgs/ColorRGBA  (latched) what the LEDs show, for RViz and the app

`driver: real` uses SPI (enable it: sudo raspi-config -> Interface -> SPI) and
GPIO 18; `driver: fake` only publishes the state (simulation).
"""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA, String

from rasptank_core.signals import parse_tune

from .drivers import FakeSignals, RealSignals

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SignalsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('signals_node', **kwargs)
        p = self.declare_parameter
        driver = p('driver', 'fake').value
        self.count = p('led_count', 8).value
        if driver == 'real':
            self.hw = RealSignals(p('spi_bus', 0).value, p('spi_device', 0).value,
                                  p('led_order', 'GRB').value, p('buzzer_pin', 18).value)
        elif driver == 'fake':
            self.hw = FakeSignals()
        else:
            raise ValueError(f'unknown driver {driver!r} (real or fake)')
        self.tune_id = 0
        self.lock = threading.Lock()
        self.state_pub = self.create_publisher(ColorRGBA, 'leds/state', LATCHED)
        self.create_subscription(ColorRGBA, 'leds', self.on_leds, 10)
        self.create_subscription(String, 'buzzer', self.on_buzzer, 10)
        self.create_subscription(Bool, 'emergency_stop', self.on_stop, LATCHED)
        self.set_color(ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0))
        self.get_logger().info(f'LEDs and buzzer ready (driver {driver})')

    def set_color(self, c):
        k = max(0.0, min(1.0, c.a if c.a > 0 else 1.0)) * 255
        rgb = tuple(int(max(0.0, min(1.0, v)) * k) for v in (c.r, c.g, c.b))
        with self.lock:
            self.hw.show([rgb] * self.count)
        self.state_pub.publish(c)

    def on_leds(self, msg):
        self.set_color(msg)

    def on_stop(self, msg):
        if msg.data:
            self.tune_id += 1
            self.hw.stop()
            self.set_color(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))

    def on_buzzer(self, msg):
        try:
            tune = parse_tune(msg.data)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self.tune_id += 1
        my_id = self.tune_id
        threading.Thread(target=self.play, args=(tune, my_id), daemon=True).start()

    def play(self, tune, my_id):
        for frequency, seconds in tune:
            if my_id != self.tune_id:
                return                         # a newer tune (or a stop) replaced it
            if frequency is None:
                self.hw.stop()
            else:
                self.hw.play(frequency)
            time.sleep(seconds)
        if my_id == self.tune_id:
            self.hw.stop()

    def destroy_node(self):
        self.tune_id += 1
        self.hw.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SignalsNode()
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
