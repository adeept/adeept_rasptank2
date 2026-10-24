"""Small geometry helpers (rigid transforms, rays, segments) without ROS."""
import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_to_quat(yaw):
    """(x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Transform:
    """Rigid transform p_parent = R @ p_child + t."""

    def __init__(self, rotation=None, translation=None):
        self.R = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
        self.t = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)

    @classmethod
    def from_quat(cls, translation, quat):
        return cls(quat_to_matrix(*quat), translation)

    @classmethod
    def from_pose2d(cls, x, y, yaw, z=0.0):
        c, s = math.cos(yaw), math.sin(yaw)
        return cls([[c, -s, 0], [s, c, 0], [0, 0, 1]], [x, y, z])

    def apply(self, points):
        """Transform one point (3,) or several (N, 3)."""
        p = np.asarray(points, dtype=float)
        return p @ self.R.T + self.t

    def rotate(self, vectors):
        return np.asarray(vectors, dtype=float) @ self.R.T

    def inverse(self):
        return Transform(self.R.T, -self.R.T @ self.t)

    def __matmul__(self, other):
        return Transform(self.R @ other.R, self.R @ other.t + self.t)


def ray_to_plane_z(origin, direction, z):
    """Point where a ray hits the plane z = const, or None if it does not."""
    dz = direction[2]
    if abs(dz) < 1e-9:
        return None
    s = (z - origin[2]) / dz
    if s <= 0:
        return None
    return np.asarray(origin, dtype=float) + s * np.asarray(direction, dtype=float)


def point_segment_distance(p, a, b):
    """Distance in the plane from point p to segment ab."""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        return math.hypot(px - ax, py - ay)
    s = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - ax - s * dx, py - ay - s * dy)


def ray_segment_distance(origin, angle, a, b):
    """Distance along a 2D ray to segment ab, or None if it misses."""
    ox, oy = origin
    rx, ry = math.cos(angle), math.sin(angle)
    ax, ay = a
    sx, sy = b[0] - ax, b[1] - ay
    denom = rx * sy - ry * sx
    if abs(denom) < 1e-12:
        return None
    qx, qy = ax - ox, ay - oy
    t = (qx * sy - qy * sx) / denom      # along the ray
    u = (qx * ry - qy * rx) / denom      # along the segment
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def box_segments(cx, cy, yaw, size_x, size_y):
    """The four edges of a rectangle in the plane."""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = size_x / 2.0, size_y / 2.0
    corners = []
    for lx, ly in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)):
        corners.append((cx + c * lx - s * ly, cy + s * lx + c * ly))
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
