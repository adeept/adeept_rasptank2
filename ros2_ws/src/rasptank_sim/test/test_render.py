"""The synthetic camera and the real detector work together."""
import math

import pytest

from rasptank_core.geometry import ray_to_plane_z
from rasptank_core.vision import (ArucoDetector, aruco_dictionary, bbox_to_point, detect_color,
                                  intrinsics_from_hfov, marker_pose)
from rasptank_sim.dataset import main as dataset_main
from rasptank_sim.render import Renderer, camera_transform
from rasptank_sim.world import World

K = intrinsics_from_hfov(640, 480, 62.2)


def world(cube=(0.6, 0.1), goal=(1.2, 0.4, math.pi)):
    return World({'arena': {'center': [0.5, 0.0], 'size': [3.0, 3.0]},
                  'cube': {'x': cube[0], 'y': cube[1]},
                  'goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2]}})


def test_cube_position_from_the_floor_plane():
    w = world()
    T = camera_transform(0.0, 0.0, 0.0, 0.35)
    img = Renderer(w).render(T, K, 640, 480, noise=2.0)
    boxes = detect_color(img)
    assert boxes, 'red cube not found'
    x, y, bw, bh, _ = boxes[0]
    p = bbox_to_point(x + bw / 2, y + bh / 2, bw, K, 0.04)
    hit = ray_to_plane_z(T.t, T.rotate(p), 0.02)          # as the mission does
    assert hit[0] == pytest.approx(0.6, abs=0.03)
    assert hit[1] == pytest.approx(0.1, abs=0.02)


def test_marker_pose_in_the_world():
    w = world()
    T = camera_transform(0.4, 0.2, 0.3, 0.15)
    img = Renderer(w).render(T, K, 640, 480)
    found = ArucoDetector(aruco_dictionary()).detect(img)
    assert [i for i, _ in found] == [0]
    _, tvec = marker_pose(found[0][1], 0.08, K)
    p = T.apply(tvec)
    assert p == pytest.approx([1.2, 0.4, w.goal.marker_center_z], abs=0.02)


def test_board_is_not_hidden_by_the_wall_behind():
    # regression: the painter's algorithm drew the long wall over the board
    w = World({'arena': {'center': [0.5, 0.2], 'size': [2.4, 2.2]},
               'goal': {'x': -0.45, 'y': 0.9, 'yaw': -0.6}})
    x, y = 0.48, -0.16
    yaw = math.atan2(0.9 - y, -0.45 - x)
    img = Renderer(w).render(camera_transform(x, y, yaw, 0.15), K, 640, 480)
    assert [i for i, _ in ArucoDetector(aruco_dictionary()).detect(img)] == [0]


def test_nothing_behind_the_camera():
    w = world(cube=(-0.6, 0.0))
    img = Renderer(w).render(camera_transform(0.0, 0.0, 0.0, 0.35), K, 640, 480)
    assert detect_color(img) == []


def test_dataset_generator(tmp_path, monkeypatch):
    monkeypatch.setattr('sys.argv', ['dataset_sim', '--out', str(tmp_path), '--images', '6'])
    dataset_main()
    images = list((tmp_path / 'images').rglob('*.jpg'))
    labels = list((tmp_path / 'labels').rglob('*.txt'))
    assert len(images) == 6 and len(labels) == 6
    rows = [line.split() for f in labels for line in f.read_text().splitlines()]
    assert rows, 'no object labelled in any image'
    for row in rows:
        assert row[0] in ('0', '1')
        assert all(0.0 <= float(v) <= 1.0 for v in row[1:])
    assert 'names' in (tmp_path / 'data.yaml').read_text()
