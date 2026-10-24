"""Camera of the RaspTank, published as JPEG to save WiFi bandwidth.

Publishes:
  /camera/image/compressed   sensor_msgs/CompressedImage (jpeg)
  /camera/camera_info        sensor_msgs/CameraInfo

`backend`:
  picamera2  the Raspberry Pi camera through libcamera (Raspberry Pi OS)
  opencv     cv2.VideoCapture(`source`): a device number ("0"), a file or
             a stream URL
  tcp        JPEG frames from scripts/pi/camara_host.py, which runs on
             Raspberry Pi OS with picamera2 when ROS runs in Docker;
             `source` is "host:port" (see docker/README.md)

The intrinsics come from `camera_matrix` / `distortion` (calibrate the
camera, see ROS2.md) or, if those are empty, from `hfov` (degrees).
"""
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
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
        self.tcp_frame = None
        if self.backend == 'tcp':
            host, _, port = source.rpartition(':')
            self.tcp_address = (host or '127.0.0.1', int(port or 8555))
            threading.Thread(target=self.tcp_reader, daemon=True).start()
        elif self.backend == 'picamera2':
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
            raise ValueError(f'unknown backend {self.backend!r} (picamera2, opencv or tcp)')

        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', 2)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 2)
        self.create_timer(1.0 / fps, self.grab)
        self._failures = 0
        self.get_logger().info(f'camera ready ({self.backend}, {self.width}x{self.height})')

    def tcp_reader(self):
        """Keeps the last JPEG sent by camara_host.py (4-byte length + data)."""
        def recv(sock, n):
            data = b''
            while len(data) < n:
                chunk = sock.recv(n - len(data))
                if not chunk:
                    raise ConnectionError('closed')
                data += chunk
            return data

        while rclpy.ok():
            try:
                with socket.create_connection(self.tcp_address, timeout=5.0) as sock:
                    self.get_logger().info(f'connected to the camera at {self.tcp_address}')
                    while rclpy.ok():
                        size, = struct.unpack('>I', recv(sock, 4))
                        self.tcp_frame = recv(sock, size)
            except (OSError, ConnectionError, struct.error) as e:
                self.get_logger().warn(f'camera stream {self.tcp_address}: {e}; retrying',
                                       throttle_duration_sec=10.0)
                time.sleep(1.0)

    def read(self):
        if self.backend == 'tcp':
            jpeg, self.tcp_frame = self.tcp_frame, None
            if jpeg is None:
                return None
            return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise       # a real error; otherwise a callback was cut by the shutdown
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
