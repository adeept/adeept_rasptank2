"""Synthetic camera for the light simulator.

Draws the world (floor, line, goal area, walls, obstacles, cube, goal board
with its ArUco marker) from a camera pose, with flat shaded polygons and the
painter's algorithm. It is far from photorealistic, but the cube is red and
the marker is a real ArUco marker, so the same detector that runs on the
real robot also works in simulation.
"""
import math

import cv2
import numpy as np

from rasptank_core.vision import aruco_dictionary, aruco_marker_image

NEAR = 0.01
FLOOR = (150, 150, 150)
BACKGROUND = (205, 210, 215)
WALL = (185, 195, 200)
LINE = (25, 25, 25)
GOAL_AREA = (120, 175, 120)
BOARD = (245, 245, 245)
LIGHT = np.array([0.4, 0.3, 0.86]) / np.linalg.norm([0.4, 0.3, 0.86])


def _clip_near(points):
    """Sutherland-Hodgman against z >= NEAR (camera frame)."""
    out = []
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        a_in, b_in = a[2] >= NEAR, b[2] >= NEAR
        if a_in:
            out.append(a)
        if a_in != b_in:
            s = (NEAR - a[2]) / (b[2] - a[2])
            out.append(a + s * (b - a))
    return out


def _clip_rect(poly, x0, y0, x1, y1):
    """Clip a 2D convex polygon to a rectangle (keeps cv2 coordinates small)."""
    for axis, limit, keep_greater in ((0, x0, True), (0, x1, False), (1, y0, True), (1, y1, False)):
        if not poly:
            return poly
        out = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            a_in = a[axis] >= limit if keep_greater else a[axis] <= limit
            b_in = b[axis] >= limit if keep_greater else b[axis] <= limit
            if a_in:
                out.append(a)
            if a_in != b_in:
                s = (limit - a[axis]) / (b[axis] - a[axis])
                out.append(a + s * (b - a))
        poly = out
    return poly


class Face:
    def __init__(self, points, color, normal=None, two_sided=False, texture=None):
        self.points = np.asarray(points, dtype=float)
        self.color = np.asarray(color, dtype=float)
        self.normal = None if normal is None else np.asarray(normal, dtype=float)
        self.two_sided = two_sided
        self.texture = texture       # (image, 4 corner points) drawn on top


def _box_faces(cx, cy, yaw, sx, sy, z0, z1, color):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for lx, ly in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2)):
        corners.append((cx + c * lx - s * ly, cy + s * lx + c * ly))
    faces = []
    for i in range(4):
        (ax, ay), (bx, by) = corners[i], corners[(i + 1) % 4]
        normal = np.array([by - ay, -(bx - ax), 0.0])
        normal /= np.linalg.norm(normal)
        faces.append(Face([(ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1)], color, normal))
    faces.append(Face([(x, y, z1) for x, y in corners], color, (0.0, 0.0, 1.0)))
    return faces


class Renderer:
    def __init__(self, world, marker_dictionary='DICT_4X4_50'):
        self.world = world
        self.marker = None
        if world.goal is not None:
            img = aruco_marker_image(aruco_dictionary(marker_dictionary), world.goal.marker_id, 120)
            self.marker = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # ------------------------------------------------------------ scene

    def floor_faces(self):
        w = self.world
        faces = []
        for a, b in w.line_segments():
            d = np.array([b[0] - a[0], b[1] - a[1]])
            length = np.linalg.norm(d)
            if length == 0:
                continue
            nx, ny = -d[1] / length * w.line_width / 2, d[0] / length * w.line_width / 2
            ex, ey = d[0] / length * w.line_width / 2, d[1] / length * w.line_width / 2
            faces.append(Face([(a[0] - nx - ex, a[1] - ny - ey, 0), (b[0] - nx + ex, b[1] - ny + ey, 0),
                               (b[0] + nx + ex, b[1] + ny + ey, 0), (a[0] + nx - ex, a[1] + ny - ey, 0)],
                              LINE, two_sided=True))
        if w.goal is not None:
            gx, gy = w.goal.area
            r = w.goal.area_radius
            faces.append(Face([(gx + r * math.cos(t), gy + r * math.sin(t), 0)
                               for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)],
                              GOAL_AREA, two_sided=True))
        return faces

    def wall_faces(self):
        w = self.world
        return [Face([(a[0], a[1], 0), (b[0], b[1], 0), (b[0], b[1], w.wall_height),
                      (a[0], a[1], w.wall_height)], WALL, two_sided=True)
                for a, b in w.wall_segments()]

    def solid_faces(self):
        w = self.world
        faces = []
        for o in w.obstacles:
            faces += _box_faces(o.x, o.y, o.yaw, o.size[0], o.size[1], 0.0, o.height, o.color)
        if w.cube is not None:
            c = w.cube
            z0 = c.position[2] - c.size / 2
            faces += _box_faces(c.position[0], c.position[1], c.yaw, c.size, c.size,
                                z0, z0 + c.size, c.color)
            faces.append(Face(_box_faces(c.position[0], c.position[1], c.yaw, c.size, c.size,
                                         z0, z0, c.color)[4].points[::-1], c.color, (0, 0, -1.0)))
        g = w.goal
        if g is not None:
            (ax, ay), (bx, by) = g.segment()
            normal = (math.cos(g.yaw), math.sin(g.yaw), 0.0)
            # front (with the marker) and back of the board
            front = Face([(ax, ay, 0), (bx, by, 0), (bx, by, g.board[1]), (ax, ay, g.board[1])],
                         BOARD, normal)
            h = g.marker_size / 2
            ux, uy = (bx - ax) / g.board[0], (by - ay) / g.board[0]   # along the board
            zc = g.marker_center_z
            # marker corners TL, TR, BR, BL as seen from the front: +u is right
            front.texture = (self.marker, np.array([
                (g.x - ux * h, g.y - uy * h, zc + h), (g.x + ux * h, g.y + uy * h, zc + h),
                (g.x + ux * h, g.y + uy * h, zc - h), (g.x - ux * h, g.y - uy * h, zc - h)]))
            faces.append(front)
            faces.append(Face(front.points[::-1], (110, 110, 110), -np.asarray(normal)))
        return faces

    # ------------------------------------------------------------ drawing

    def render(self, T_world_cam, K, width, height, noise=0.0):
        """BGR image seen by a camera (optical frame pose in the world)."""
        T = T_world_cam.inverse()
        cam_pos = T_world_cam.t
        img = np.empty((height, width, 3), np.uint8)
        img[:] = BACKGROUND
        rect = (-width, -height, 2 * width, 2 * height)
        big = 30.0
        floor = Face([(-big, -big, 0), (big, -big, 0), (big, big, 0), (-big, big, 0)], FLOOR,
                     two_sided=True)
        self._draw(img, floor, T, K, rect, 1.0)
        for face in self.floor_faces():
            self._draw(img, face, T, K, rect, 1.0)
        # the walls surround everything else: drawing them first avoids the
        # painter's algorithm putting a long wall in front of what it hides
        for face in self.wall_faces():
            self._draw(img, face, T, K, rect, 1.0)
        solids = []
        for face in self.solid_faces():
            if face.normal is not None and not face.two_sided:
                if np.dot(face.normal, cam_pos - face.points.mean(axis=0)) <= 0:
                    continue
            shade = 1.0
            if face.normal is not None:
                shade = 0.6 + 0.4 * abs(float(np.dot(face.normal, LIGHT)))
            depth = np.linalg.norm(face.points.mean(axis=0) - cam_pos)
            solids.append((depth, face, shade))
        solids.sort(key=lambda s: -s[0])
        for _, face, shade in solids:
            self._draw(img, face, T, K, rect, shade)
        if noise > 0:
            n = np.random.normal(0.0, noise, img.shape)
            img = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)
        return img

    def _project(self, cam_points, K):
        z = cam_points[:, 2]
        u = K[0, 0] * cam_points[:, 0] / z + K[0, 2]
        v = K[1, 1] * cam_points[:, 1] / z + K[1, 2]
        return np.stack([u, v], axis=1)

    def _draw(self, img, face, T, K, rect, shade):
        cam = T.apply(face.points)
        clipped = _clip_near(list(cam))
        if len(clipped) < 3:
            return
        poly = _clip_rect(list(self._project(np.array(clipped), K)), *rect)
        if len(poly) < 3:
            return
        color = tuple(float(c) for c in np.clip(face.color * shade, 0, 255))
        cv2.fillConvexPoly(img, np.round(np.array(poly) * 16).astype(np.int32), color,
                           lineType=cv2.LINE_AA, shift=4)
        if face.texture is not None:
            self._draw_texture(img, face.texture, T, K)

    def _draw_texture(self, img, texture, T, K):
        tex, corners = texture
        cam = T.apply(corners)
        if np.any(cam[:, 2] < NEAR):
            return
        dst = self._project(cam, K).astype(np.float32)
        h, w = img.shape[:2]
        if np.any(np.abs(dst) > 4 * max(w, h)):
            return
        th, tw = tex.shape[:2]
        src = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]])
        H = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(tex, H, (w, h), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full((th, tw), 255, np.uint8), H, (w, h))
        img[mask > 127] = warped[mask > 127]


OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def camera_transform(x, y, yaw, tilt, camera_x=0.10, camera_z=0.095):
    """Pose of camera_optical_frame in the world for a robot pose.

    Same chain as the URDF (base_footprint -> camera_link -> optical);
    the simulator node uses TF instead, this is for tests and datasets.
    """
    from rasptank_core.geometry import Transform
    c, s = math.cos(tilt), math.sin(tilt)
    tilt_R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    mount = Transform(tilt_R, [camera_x, 0.0, camera_z])
    return Transform.from_pose2d(x, y, yaw) @ mount @ Transform(OPTICAL)
