import cv2
import numpy as np
import pytest

from rasptank_core.vision import decode_yolo
from rasptank_perception.detector_node import rvec_to_quat


def test_rvec_to_quat_matches_rodrigues():
    rvec = np.array([0.3, -1.2, 0.5])
    x, y, z, w = rvec_to_quat(rvec)
    R, _ = cv2.Rodrigues(rvec)
    R2 = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    assert np.allclose(R, R2, atol=1e-6)


def test_decode_yolo_output():
    # 2 classes, 3 candidates: two overlapping cubes and one goal, in a 320 input
    out = np.zeros((1, 6, 3), np.float32)
    out[0, :, 0] = [100, 100, 40, 40, 0.9, 0.0]
    out[0, :, 1] = [102, 101, 40, 40, 0.8, 0.0]     # suppressed by NMS
    out[0, :, 2] = [200, 150, 30, 60, 0.0, 0.7]
    found = decode_yolo(out, scale=0.5, conf=0.5)
    assert len(found) == 2
    cls, score, x, y, w, h = sorted(found)[0]
    assert cls == 0 and score == pytest.approx(0.9)
    assert (x, y, w, h) == pytest.approx((160, 160, 80, 80))   # back to image pixels
    assert sorted(found)[1][0] == 1
    assert decode_yolo(np.zeros((1, 6, 3), np.float32), 1.0) == []
