"""World of the light simulator: arena, obstacles, floor line, cube and goal.

Pure Python (no ROS), so the sensors and the renderer can be tested and
used to generate datasets. Coordinates are in the `odom` frame, where the
robot starts at (0, 0) looking along +x.

World YAML (see rasptank_bringup/worlds/):

    arena: {center: [0.6, 0.0], size: [2.4, 2.4], wall_height: 0.15}
    obstacles:
      - {x: 0.6, y: -0.7, yaw: 0.0, size: [0.2, 0.2], height: 0.12}
    line: {width: 0.019, closed: true, points: [[0, 0], [1, 0], ...]}
    cube: {x: 0.9, y: 0.3, size: 0.04, color: [40, 40, 200]}      # BGR
    goal: {x: -0.3, y: 0.9, yaw: -1.57, marker_id: 0, marker_size: 0.08,
           board: [0.14, 0.16], area_radius: 0.15}

Several cubes and goals (sorting): `cubes:` and `goals:` lists instead,
each cube with a `name` (its class, e.g. cubo_azul), and
`sorting: {cubo_rojo: 0, cubo_azul: 1}` saying which marker id each class
goes to.

The goal is an ArUco marker on a vertical board standing on the floor;
`yaw` is the direction the marker faces. The cube must end inside the
circle of `area_radius` in front of the board.
"""
import math
import random

import numpy as np

from rasptank_core.geometry import box_segments, point_segment_distance, ray_segment_distance


class Box:
    def __init__(self, x, y, yaw, size, height, color=(90, 110, 140)):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.size = (float(size[0]), float(size[1]))
        self.height = float(height)
        self.color = tuple(int(c) for c in color)

    def segments(self):
        return box_segments(self.x, self.y, self.yaw, *self.size)

    def contains(self, px, py, margin=0.0):
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        lx = c * (px - self.x) - s * (py - self.y)
        ly = s * (px - self.x) + c * (py - self.y)
        return abs(lx) <= self.size[0] / 2 + margin and abs(ly) <= self.size[1] / 2 + margin


class Cube:
    def __init__(self, x, y, size=0.04, color=(40, 40, 200), yaw=0.0, name='cubo'):
        self.name = name
        self.size = float(size)
        self.position = np.array([float(x), float(y), self.size / 2])
        self.yaw = float(yaw)
        self.color = tuple(int(c) for c in color)
        self.held = False


class Goal:
    def __init__(self, x, y, yaw, marker_id=0, marker_size=0.08, board=(0.14, 0.16),
                 area_radius=0.15, area_offset=0.11):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.marker_id = int(marker_id)
        self.marker_size = float(marker_size)
        self.board = (float(board[0]), float(board[1]))
        self.area_radius = float(area_radius)
        # the cube should end on this circle, in front of the board
        self.area = (self.x + area_offset * math.cos(self.yaw),
                     self.y + area_offset * math.sin(self.yaw))

    @property
    def marker_center_z(self):
        return self.board[1] / 2.0

    def segment(self):
        """The board seen from above (width along the marker plane)."""
        dx, dy = -math.sin(self.yaw) * self.board[0] / 2, math.cos(self.yaw) * self.board[0] / 2
        return (self.x - dx, self.y - dy), (self.x + dx, self.y + dy)


class World:
    def __init__(self, config):
        arena = config.get('arena', {})
        self.arena_center = tuple(arena.get('center', [0.6, 0.0]))
        self.arena_size = tuple(arena.get('size', [2.4, 2.4]))
        self.wall_height = float(arena.get('wall_height', 0.15))
        self.obstacles = [Box(o['x'], o['y'], o.get('yaw', 0.0), o['size'],
                              o.get('height', 0.12), o.get('color', (90, 110, 140)))
                          for o in config.get('obstacles', [])]
        line = config.get('line') or {}
        self.line_width = float(line.get('width', 0.019))
        points = [tuple(p) for p in line.get('points', [])]
        if line.get('closed', False) and points:
            points.append(points[0])
        self.line = points
        cubes = config.get('cubes') or ([config['cube']] if config.get('cube') else [])
        self.cubes = [Cube(c['x'], c['y'], c.get('size', 0.04), c.get('color', (40, 40, 200)),
                           c.get('yaw', 0.0), c.get('name', 'cubo')) for c in cubes]
        goals = config.get('goals') or ([config['goal']] if config.get('goal') else [])
        self.goals = [Goal(g['x'], g['y'], g.get('yaw', 0.0), g.get('marker_id', 0),
                           g.get('marker_size', 0.08), g.get('board', (0.14, 0.16)),
                           g.get('area_radius', 0.15)) for g in goals]
        self.sorting = dict(config.get('sorting') or {})

    @property
    def cube(self):
        """The first cube (worlds with one cube)."""
        return self.cubes[0] if self.cubes else None

    @property
    def goal(self):
        return self.goals[0] if self.goals else None

    # ------------------------------------------------------------ geometry

    def wall_segments(self):
        cx, cy = self.arena_center
        return box_segments(cx, cy, 0.0, *self.arena_size)

    def line_segments(self):
        return list(zip(self.line[:-1], self.line[1:]))

    def inside_arena(self, x, y, margin=0.0):
        cx, cy = self.arena_center
        return (abs(x - cx) <= self.arena_size[0] / 2 - margin
                and abs(y - cy) <= self.arena_size[1] / 2 - margin)

    # ------------------------------------------------------------ sensors

    def ultrasonic(self, x, y, yaw, height, max_range=2.0, fov=math.radians(30), rays=7,
                   noise=0.0):
        """Closest echo in the cone, or max_range."""
        segments = list(self.wall_segments())
        for o in self.obstacles:
            if o.height >= height:
                segments += o.segments()
        segments += [g.segment() for g in self.goals]
        for cube in self.cubes:
            if cube.position[2] - cube.size / 2 <= height <= cube.position[2] + cube.size / 2:
                # only when it is at the height of the sensor: not on the floor
                # below it (4 cm cube) nor up in the gripper
                cx, cy = cube.position[:2]
                segments += box_segments(cx, cy, cube.yaw, cube.size, cube.size)
        best = max_range
        for i in range(rays):
            angle = yaw + (i / (rays - 1) - 0.5) * fov if rays > 1 else yaw
            for a, b in segments:
                d = ray_segment_distance((x, y), angle, a, b)
                if d is not None and d < best:
                    best = d
        if noise and best < max_range:
            best += random.gauss(0.0, noise)
        return max(0.02, min(max_range, best))

    def on_line(self, x, y):
        half = self.line_width / 2.0
        return any(point_segment_distance((x, y), a, b) <= half for a, b in self.line_segments())

    def collides(self, x, y, yaw, length=0.22, width=0.15):
        """Whether the robot footprint touches a wall or an obstacle."""
        c, s = math.cos(yaw), math.sin(yaw)
        for fx in (-0.5, 0.0, 0.5):
            for fy in (-0.5, 0.0, 0.5):
                if fx == 0.0 and fy == 0.0:
                    continue
                px = x + c * fx * length - s * fy * width
                py = y + s * fx * length + c * fy * width
                if not self.inside_arena(px, py):
                    return True
                if any(o.contains(px, py) for o in self.obstacles):
                    return True
        return False

    # ------------------------------------------------------------ cubes

    @property
    def held(self):
        return next((c for c in self.cubes if c.held), None)

    def try_grasp(self, gripper_xyz, radius=0.03):
        """Grasp the cube closest to the gripper centre, if close enough."""
        if self.held is not None or not self.cubes:
            return False
        p = np.asarray(gripper_xyz)
        cube = min(self.cubes, key=lambda c: np.linalg.norm(p - c.position))
        if np.linalg.norm(p - cube.position) <= radius:
            cube.held = True
            return True
        return False

    def carry(self, gripper_xyz):
        cube = self.held
        if cube is not None:
            cube.position = np.asarray(gripper_xyz, dtype=float)

    def release(self):
        cube = self.held
        if cube is None:
            return False
        cube.held = False
        cube.position = np.array([cube.position[0], cube.position[1], cube.size / 2])
        return True

    def goal_of(self, cube):
        """The goal whose area the cube is on, or None."""
        if cube.held:
            return None
        for g in self.goals:
            gx, gy = g.area
            if math.hypot(cube.position[0] - gx, cube.position[1] - gy) <= g.area_radius:
                return g
        return None

    def cube_in_goal(self):
        cube = self.cube
        return cube is not None and self.goal is not None and self.goal_of(cube) is self.goal

    def sorted_count(self):
        """(cubes on the goal of their class, total) according to `sorting`."""
        right = 0
        for c in self.cubes:
            g = self.goal_of(c)
            if g is not None and self.sorting.get(c.name, g.marker_id) == g.marker_id:
                right += 1
        return right, len(self.cubes)
