import math

import pytest

from rasptank_sim.world import World

CONFIG = {
    'arena': {'center': [0.0, 0.0], 'size': [2.0, 2.0]},
    'obstacles': [{'x': 0.5, 'y': 0.5, 'size': [0.2, 0.2], 'height': 0.12}],
    'line': {'points': [[-0.5, 0.0], [0.5, 0.0]], 'width': 0.02},
    'cube': {'x': 0.3, 'y': -0.3},
    'goal': {'x': -0.8, 'y': 0.0, 'yaw': 0.0},
}


def test_ultrasonic_sees_walls_obstacles_not_low_cube():
    w = World(CONFIG)
    assert w.ultrasonic(0.0, 0.0, 0.0, 0.06, noise=0) == pytest.approx(1.0, abs=0.01)
    # the obstacle face is at x = 0.4
    assert w.ultrasonic(0.0, 0.5, 0.0, 0.06, noise=0) == pytest.approx(0.4, abs=0.01)
    # the cube (4 cm) is below the sensor (6 cm): it sees the wall behind
    assert w.ultrasonic(0.0, -0.3, 0.0, 0.06, noise=0) == pytest.approx(1.0, abs=0.01)
    # nothing within range
    assert w.ultrasonic(0.0, 0.0, 0.0, 0.06, max_range=0.5, noise=0) == 0.5


def test_line_and_collision():
    w = World(CONFIG)
    assert w.on_line(0.0, 0.005)
    assert not w.on_line(0.0, 0.02)
    assert not w.collides(0.0, 0.0, 0.0)
    assert w.collides(0.9, 0.0, 0.0)          # front outside the arena
    assert w.collides(0.35, 0.5, 0.0)         # nose inside the obstacle


def test_grasp_carry_release_goal():
    w = World(CONFIG)
    assert not w.try_grasp((0.0, 0.0, 0.02))
    assert w.try_grasp((0.31, -0.3, 0.025))
    w.carry((-0.68, 0.0, 0.2))
    assert not w.cube_in_goal()               # still held
    assert w.release()
    assert w.cube.position[2] == pytest.approx(0.02)
    assert w.cube_in_goal()
    assert not w.release()


def test_goal_area_is_in_front_of_the_board():
    w = World(CONFIG)
    ax, ay = w.goal.area
    assert ax > w.goal.x and ay == pytest.approx(0.0)
    assert math.hypot(ax - w.goal.x, ay - w.goal.y) == pytest.approx(0.11)
