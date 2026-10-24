"""Detects the cube and the goal in the camera images.

Subscribes:
  /camera/image/compressed   sensor_msgs/CompressedImage
  /camera/camera_info        sensor_msgs/CameraInfo
Publishes:
  /detections                vision_msgs/Detection2DArray
  /detections/image          sensor_msgs/Image (bgr8, boxes drawn) for RViz

Each detection has a `class_id`, the box in pixels and the 3D position in
the camera optical frame (header.frame_id). Classes: "cubo" (or the ones of
`color_classes` / the YOLO model), "meta" for the marker `goal_marker_id`,
and "meta_<id>" for every ArUco marker seen (sorting uses several goals):
  cubo  from the box size and `cube_size` (approximate distance; the
        mission corrects it with the floor plane)
  meta  from the ArUco marker corners with solvePnP (precise)

`cube_detector`:
  color  coloured blobs in HSV: `color_classes` (default [cubo]), each with
         its ranges in `hsv_<class>` (`hsv_ranges` for "cubo"); up to
         `max_per_class` blobs per class, largest first
  yolo   ONNX model trained with tools/yolo (classes `yolo_classes`)
The goal is always the ArUco marker `goal_marker_id`.

Runs on the PC: the Raspberry Pi 4 only sends the JPEG images.
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from rasptank_core.vision import (ArucoDetector, YoloOnnx, aruco_dictionary, bbox_to_point,
                                  detect_color, marker_pose)


def rvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return x, y, z, w


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        p = self.declare_parameter
        self.cube_detector = p('cube_detector', 'color').value
        red = [0, 120, 70, 10, 255, 255, 170, 120, 70, 180, 255, 255]
        self.color_classes = {}
        for name in p('color_classes', ['cubo']).value:
            flat = list(p('hsv_ranges' if name == 'cubo' else f'hsv_{name}', red).value)
            self.color_classes[name] = [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6]))
                                        for i in range(0, len(flat) - 5, 6)]
        self.max_per_class = p('max_per_class', 3).value
        self.min_area = p('min_area', 80).value
        self.cube_size = p('cube_size', 0.04).value
        self.goal_id = p('goal_marker_id', 0).value
        self.marker_size = p('marker_size', 0.08).value
        self.aruco = ArucoDetector(aruco_dictionary(p('aruco_dictionary', 'DICT_4X4_50').value))
        self.debug = p('debug_image', True).value
        self.yolo = None
        self.yolo_classes = list(p('yolo_classes', ['cubo', 'meta']).value)
        if self.cube_detector == 'yolo':
            model = p('yolo_model', '').value
            if not model:
                raise ValueError('cube_detector yolo needs the yolo_model parameter (.onnx)')
            self.yolo = YoloOnnx(model, p('yolo_input', 320).value, p('yolo_confidence', 0.5).value)
        elif self.cube_detector != 'color':
            raise ValueError(f'unknown cube_detector {self.cube_detector!r} (color or yolo)')

        self.K = None
        self.dist = np.zeros(5)
        self.det_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.image_pub = self.create_publisher(Image, 'detections/image', 2)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, 10)
        self.create_subscription(CompressedImage, 'camera/image/compressed', self.on_image,
                                 qos_profile_sensor_data)
        self._times = []
        self.get_logger().info(f'detector ready (cube: {self.cube_detector}, goal: ArUco '
                               f'{self.goal_id})')

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        if len(msg.d) >= 4:
            self.dist = np.array(list(msg.d)[:5] + [0.0] * (5 - len(msg.d[:5])), dtype=float)

    def detection(self, header, cls, score, x, y, w, h, position, quat=None):
        d = Detection2D()
        d.header = header
        d.bbox.center.position.x = float(x + w / 2)
        d.bbox.center.position.y = float(y + h / 2)
        d.bbox.size_x, d.bbox.size_y = float(w), float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = cls
        hyp.hypothesis.score = float(score)
        hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = \
            (float(v) for v in position)
        if quat is not None:
            o = hyp.pose.pose.orientation
            o.x, o.y, o.z, o.w = (float(v) for v in quat)
        d.results.append(hyp)
        d.id = cls
        return d

    def on_image(self, msg):
        if self.K is None:
            self.get_logger().warn('waiting for camera_info', throttle_duration_sec=5.0)
            return
        t0 = time.monotonic()
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        out = Detection2DArray()
        out.header = msg.header
        boxes = []     # for the debug image: (label, x, y, w, h, colour)

        if self.yolo is None:
            for name, ranges in self.color_classes.items():
                for x, y, w, h, area in detect_color(img, ranges, self.min_area)[:self.max_per_class]:
                    fill = area / float(w * h)
                    pos = bbox_to_point(x + w / 2, y + h / 2, w, self.K, self.cube_size)
                    out.detections.append(self.detection(msg.header, name, min(1.0, fill), x, y,
                                                         w, h, pos))
                    boxes.append((name, x, y, w, h, (0, 0, 255)))
        else:
            count = {}
            for cls, score, x, y, w, h in self.yolo.detect(img):    # best first
                name = self.yolo_classes[cls] if cls < len(self.yolo_classes) else str(cls)
                if name.startswith('meta') or count.get(name, 0) >= self.max_per_class:
                    continue                  # the goals come from the ArUco markers
                count[name] = count.get(name, 0) + 1
                pos = bbox_to_point(x + w / 2, y + h / 2, w, self.K, self.cube_size)
                out.detections.append(self.detection(msg.header, name, score, x, y, w, h, pos))
                boxes.append((f'{name} {score:.2f}', x, y, w, h, (0, 0, 255)))

        for marker_id, corners in self.aruco.detect(img):
            pose = marker_pose(corners, self.marker_size, self.K, self.dist)
            if pose is None:
                continue
            rvec, tvec = pose
            x, y = corners.min(axis=0)
            w, h = corners.max(axis=0) - corners.min(axis=0)
            names = [f'meta_{marker_id}'] + (['meta'] if marker_id == self.goal_id else [])
            for name in names:
                out.detections.append(self.detection(msg.header, name, 1.0, x, y, w, h, tvec,
                                                     rvec_to_quat(rvec)))
            boxes.append((f'meta {marker_id} {tvec[2]:.2f} m', int(x), int(y), int(w), int(h),
                          (0, 200, 0)))

        self.det_pub.publish(out)
        self._times.append(time.monotonic() - t0)
        if len(self._times) == 50:
            self.get_logger().info(f'detector: {1000 * sum(self._times) / 50:.1f} ms per image')
            self._times = []
        if self.debug and self.image_pub.get_subscription_count() > 0:
            for label, x, y, w, h, color in boxes:
                cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
                cv2.putText(img, label, (int(x), max(12, int(y) - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1, cv2.LINE_AA)
            im = Image(height=img.shape[0], width=img.shape[1], encoding='bgr8',
                       step=img.shape[1] * 3, data=img.tobytes())
            im.header = msg.header
            self.image_pub.publish(im)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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
