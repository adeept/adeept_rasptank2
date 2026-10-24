import math

import numpy as np
import pytest

from rasptank_core.geometry import (Transform, box_segments, point_segment_distance, quat_to_yaw,
                                    ray_segment_distance, ray_to_plane_z, yaw_to_quat)


def test_transform_compose_inverse():
    a = Transform.from_pose2d(1.0, 2.0, math.pi / 2)
    p = a.apply([1.0, 0.0, 0.0])
    assert p == pytest.approx([1.0, 3.0, 0.0])
    assert a.inverse().apply(p) == pytest.approx([1.0, 0.0, 0.0])
    b = a @ a.inverse()
    assert np.allclose(b.R, np.eye(3)) and np.allclose(b.t, 0)


def test_yaw_quat():
    assert quat_to_yaw(*yaw_to_quat(0.7)) == pytest.approx(0.7)
    t = Transform.from_quat([0, 0, 0], yaw_to_quat(0.7))
    assert np.allclose(t.R, Transform.from_pose2d(0, 0, 0.7).R)


def test_ray_plane():
    p = ray_to_plane_z([0, 0, 1.0], [1.0, 0, -1.0], 0.0)
    assert p == pytest.approx([1.0, 0.0, 0.0])
    assert ray_to_plane_z([0, 0, 1.0], [1.0, 0, 1.0], 0.0) is None


def test_segments():
    assert point_segment_distance((0, 1), (-1, 0), (1, 0)) == pytest.approx(1)
    assert point_segment_distance((3, 0), (-1, 0), (1, 0)) == pytest.approx(2)
    assert ray_segment_distance((0, 0), 0.0, (2, -1), (2, 1)) == pytest.approx(2)
    assert ray_segment_distance((0, 0), math.pi, (2, -1), (2, 1)) is None
    edges = box_segments(1.0, 0.0, 0.0, 0.2, 0.2)
    hits = [ray_segment_distance((0, 0), 0.0, a, b) for a, b in edges]
    assert min(h for h in hits if h is not None) == pytest.approx(0.9)
