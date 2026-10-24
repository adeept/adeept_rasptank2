"""Save camera images to build a dataset.

    ros2 run rasptank_perception capture_node --ros-args -p out_dir:=~/dataset_real -p period:=1.0

Saves one JPEG every `period` seconds (as it arrives, without decoding)
up to `max_images`. Move the robot around the cube with the teleoperation
while it runs: different distances, angles, backgrounds and light.
Then label the images (Roboflow, Label Studio, CVAT) as in tools/yolo.
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        self.out_dir = os.path.expanduser(self.declare_parameter('out_dir', '~/dataset_real').value)
        self.period = self.declare_parameter('period', 1.0).value
        self.max_images = self.declare_parameter('max_images', 500).value
        self.prefix = self.declare_parameter('prefix', 'img').value
        os.makedirs(self.out_dir, exist_ok=True)
        self.count = len([f for f in os.listdir(self.out_dir) if f.endswith('.jpg')])
        self.last = None
        self.create_subscription(CompressedImage, 'camera/image/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'saving to {self.out_dir} ({self.count} already there)')

    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is not None and now - self.last < self.period:
            return
        if self.count >= self.max_images:
            self.get_logger().info('max_images reached', throttle_duration_sec=10.0)
            return
        self.last = now
        path = os.path.join(self.out_dir, f'{self.prefix}_{self.count:05d}.jpg')
        with open(path, 'wb') as f:
            f.write(bytes(msg.data))
        self.count += 1
        self.get_logger().info(f'{path}')


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
