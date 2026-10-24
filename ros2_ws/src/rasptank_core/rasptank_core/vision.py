"""Vision helpers: colour blobs, ArUco markers and a YOLO (ONNX) detector.

All functions work on OpenCV BGR images and pinhole intrinsics K (3x3).
They support OpenCV 4.5 (Ubuntu 22.04, ROS 2 Humble) and the newer
ArUco API of OpenCV >= 4.7.
"""
import math

import cv2
import numpy as np

# Red wraps around hue 0 in HSV, so it needs two ranges.
RED_HSV = [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))]


def intrinsics_from_hfov(width, height, hfov_deg):
    """K for an ideal camera with the given horizontal field of view."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]])


def detect_color(bgr, hsv_ranges=RED_HSV, min_area=60):
    """Bounding boxes (x, y, w, h, area) of the blobs of a colour, largest first."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for low, high in hsv_ranges:
        part = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = found[0] if len(found) == 2 else found[1]
    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area >= min_area:
            x, y, w, h = cv2.boundingRect(c)
            boxes.append((x, y, w, h, area))
    boxes.sort(key=lambda b: -b[4])
    return boxes


def bbox_to_point(cx, cy, size_px, K, real_size):
    """3D point (camera optical frame) of an object of known size.

    The distance comes from the apparent size, so it is approximate; the
    direction (the ray through the pixel) is exact.
    """
    fx, fy, px, py = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = fx * real_size / max(size_px, 1.0)
    return ((cx - px) / fx * z, (cy - py) / fy * z, z)


# ---------------------------------------------------------------- ArUco

def aruco_dictionary(name='DICT_4X4_50'):
    ident = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(ident)
    return cv2.aruco.Dictionary_get(ident)


def aruco_marker_image(dictionary, marker_id, pixels):
    """Marker image (black and white, no quiet zone)."""
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, pixels)


class ArucoDetector:
    def __init__(self, dictionary):
        self.dictionary = dictionary
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        else:
            self._detector = None
            self._params = cv2.aruco.DetectorParameters_create()

    def detect(self, bgr):
        """List of (id, corners) with corners a 4x2 array (TL, TR, BR, BL)."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                      parameters=self._params)
        if ids is None:
            return []
        return [(int(i), c.reshape(4, 2)) for i, c in zip(ids.flatten(), corners)]


def marker_pose(corners, size, K, dist=None):
    """(rvec, tvec) of a square marker of side `size` in the camera frame."""
    h = size / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)
    flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, dtype=np.float64), K, dist, flags=flag)
    if not ok:
        return None
    return rvec.flatten(), tvec.flatten()


# ---------------------------------------------------------------- YOLO

class YoloOnnx:
    """YOLOv8/YOLO11 detection model exported to ONNX (Ultralytics).

    Runs with OpenCV's DNN module, so the PC needs neither PyTorch nor
    Ultralytics to use a trained model.
    """

    def __init__(self, path, input_size=320, conf=0.5, iou=0.45):
        self.net = cv2.dnn.readNetFromONNX(path)
        self.size = input_size
        self.conf = conf
        self.iou = iou

    def detect(self, bgr):
        """List of (class_index, score, x, y, w, h) in image pixels."""
        h0, w0 = bgr.shape[:2]
        scale = self.size / max(h0, w0)
        nw, nh = int(round(w0 * scale)), int(round(h0 * scale))
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[:nh, :nw] = cv2.resize(bgr, (nw, nh))
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (self.size, self.size), swapRB=True)
        self.net.setInput(blob)
        return decode_yolo(self.net.forward(), scale, self.conf, self.iou)


def decode_yolo(output, scale, conf=0.5, iou=0.45):
    """Decode an Ultralytics output (1, 4 + classes, N) with NMS."""
    pred = np.squeeze(output, 0).T          # (N, 4 + classes)
    scores = pred[:, 4:]
    cls = scores.argmax(axis=1)
    best = scores[np.arange(len(cls)), cls]
    keep = best >= conf
    pred, cls, best = pred[keep], cls[keep], best[keep]
    boxes = []
    for cx, cy, w, h in pred[:, :4]:
        boxes.append([float((cx - w / 2) / scale), float((cy - h / 2) / scale),
                      float(w / scale), float(h / scale)])
    if not boxes:
        return []
    idx = cv2.dnn.NMSBoxes(boxes, best.tolist(), conf, iou)
    idx = np.array(idx).flatten()
    return [(int(cls[i]), float(best[i]), *boxes[i]) for i in idx]
