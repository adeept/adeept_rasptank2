"""Camera of the RaspTank, published as JPEG to save WiFi bandwidth.

Publishes:
  /camera/image/compressed   sensor_msgs/CompressedImage (jpeg)
  /camera/camera_info        sensor_msgs/CameraInfo

`backend`:
  picamera2  the Raspberry Pi camera through libcamera (Raspberry Pi OS)
  opencv     cv2.VideoCapture(`source`): a device number ("0"), a file, or
             a stream URL such as "tcp://127.0.0.1:8554" (rpicam-vid on the
             host when ROS runs in Docker; see docker/README.md)

The intrinsics come from `camera_matrix` / `distortion` (calibrate the
camera, see ROS2.md) or, if those are empty, from `hfov` (degrees).
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from rasptank_core.vision import intrinsics_from_hfov


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        p = self.declare_parameter
        self.backend = p('backend', 'picamera2').value
        source = p('source', '0').value
        self.width = p('width', 640).value
        self.height = p('height', 480).value
        fps = p('fps', 10.0).value
        self.quality = p('jpeg_quality', 70).value
        self.flip = p('flip', False).value
        self.frame_id = p('frame_id', 'camera_optical_frame').value
        matrix = list(p('camera_matrix', [0.0]).value)
        dist = list(p('distortion', [0.0]).value)
        hfov = p('hfov', 62.2).value

        if len(matrix) == 9:
            K = np.array(matrix).reshape(3, 3)
        else:
            K = intrinsics_from_hfov(self.width, self.height, hfov)
        self.info = CameraInfo(width=self.width, height=self.height,
                               distortion_model='plumb_bob')
        self.info.k = K.flatten().tolist()
        self.info.d = (dist + [0.0] * 5)[:5] if len(dist) >= 4 else [0.0] * 5
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]

        self.picam = None
        self.capture = None
        if self.backend == 'picamera2':
            from picamera2 import Picamera2
            self.picam = Picamera2()
            config = self.picam.create_video_configuration(
                main={'size': (self.width, self.height), 'format': 'RGB888'})
            self.picam.configure(config)
            self.picam.start()
        elif self.backend == 'opencv':
            self.capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        else:
            raise ValueError(f'unknown backend {self.backend!r} (picamera2 or opencv)')

        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', 2)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 2)
        self.create_timer(1.0 / fps, self.grab)
        self._failures = 0
        self.get_logger().info(f'camera ready ({self.backend}, {self.width}x{self.height})')

    def read(self):
        if self.picam is not None:
            return self.picam.capture_array()       # 'RGB888' is BGR in memory
        ok, frame = self.capture.read()
        return frame if ok else None

    def grab(self):
        frame = self.read()
        if frame is None:
            self._failures += 1
            self.get_logger().warn('no camera frame', throttle_duration_sec=5.0)
            if self.capture is not None and self._failures % 20 == 0:
                time.sleep(0.5)
            return
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        if self.flip:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage(format='jpeg', data=jpeg.tobytes())
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)
        self.info.header = msg.header
        self.info_pub.publish(self.info)

    def destroy_node(self):
        if self.picam is not None:
            self.picam.stop()
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
