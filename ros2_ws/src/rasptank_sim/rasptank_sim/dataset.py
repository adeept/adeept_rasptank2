"""Generate a labelled YOLO dataset with the simulator's camera.

    ros2 run rasptank_sim dataset_sim --out ~/dataset_sim --images 600

Places the cube, the goal and some obstacles at random, puts the robot
looking at them from random poses and writes the images with their
labels in the YOLO format (classes: 0 cubo, 1 meta), plus data.yaml.
Useful to start, but a model trained only on synthetic images works
badly on the real camera: mix them with real photos (capture_node).
"""
import argparse
import math
import os
import random

import cv2
import numpy as np

from rasptank_core.vision import intrinsics_from_hfov

from .render import Renderer, camera_transform
from .world import World

CLASSES = ['cubo', 'meta']


def random_world(rng):
    obstacles = []
    for _ in range(rng.randint(0, 3)):
        obstacles.append({'x': rng.uniform(-0.8, 1.6), 'y': rng.uniform(-1.0, 1.0),
                          'yaw': rng.uniform(0, math.pi), 'size': [rng.uniform(0.08, 0.25)] * 2,
                          'height': rng.uniform(0.05, 0.15),
                          'color': [rng.randint(40, 200) for _ in range(3)]})
    red = (rng.randint(20, 70), rng.randint(20, 70), rng.randint(150, 230))
    return {
        'arena': {'center': [0.4, 0.0], 'size': [rng.uniform(2.4, 3.5), rng.uniform(2.4, 3.5)]},
        'obstacles': obstacles,
        'cube': {'x': rng.uniform(0.2, 1.0), 'y': rng.uniform(-0.4, 0.4), 'yaw': rng.uniform(0, 1.6),
                 'color': list(red)},
        'goal': {'x': rng.uniform(0.6, 1.5), 'y': rng.uniform(-0.6, 0.6),
                 'yaw': math.pi + rng.uniform(-0.6, 0.6), 'marker_id': 0},
    }


def project_bbox(points, T_world_cam, K, width, height):
    cam = T_world_cam.inverse().apply(points)
    if np.any(cam[:, 2] < 0.05):
        return None
    u = K[0, 0] * cam[:, 0] / cam[:, 2] + K[0, 2]
    v = K[1, 1] * cam[:, 1] / cam[:, 2] + K[1, 2]
    x0, x1 = np.clip([u.min(), u.max()], 0, width)
    y0, y1 = np.clip([v.min(), v.max()], 0, height)
    full = (u.max() - u.min()) * (v.max() - v.min())
    if full <= 0 or (x1 - x0) * (y1 - y0) < 0.5 * full or (x1 - x0) < 6 or (y1 - y0) < 6:
        return None
    return ((x0 + x1) / 2 / width, (y0 + y1) / 2 / height, (x1 - x0) / width, (y1 - y0) / height)


def cube_corners(cube):
    h = cube.size / 2
    c, s = math.cos(cube.yaw), math.sin(cube.yaw)
    pts = []
    for lx in (-h, h):
        for ly in (-h, h):
            for lz in (-h, h):
                pts.append(cube.position + np.array([c * lx - s * ly, s * lx + c * ly, lz]))
    return np.array(pts)


def board_corners(goal):
    (ax, ay), (bx, by) = goal.segment()
    return np.array([(ax, ay, 0), (bx, by, 0), (bx, by, goal.board[1]), (ax, ay, goal.board[1])])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', default=os.path.expanduser('~/dataset_sim'))
    parser.add_argument('--images', type=int, default=600)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--val', type=float, default=0.2, help='fraction for validation')
    parser.add_argument('--seed', type=int, default=1)
    args, _ = parser.parse_known_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    K = intrinsics_from_hfov(args.width, args.height, 62.2)
    for split in ('train', 'val'):
        os.makedirs(os.path.join(args.out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', split), exist_ok=True)
    written = 0
    while written < args.images:
        world = World(random_world(rng))
        renderer = Renderer(world)
        for _ in range(5):
            # look roughly towards the cube or the goal
            tx, ty = (world.cube.position[:2] if rng.random() < 0.6
                      else (world.goal.x, world.goal.y))
            dist, ang = rng.uniform(0.25, 1.4), rng.uniform(-math.pi, math.pi)
            x, y = tx + dist * math.cos(ang), ty + dist * math.sin(ang)
            if not world.inside_arena(x, y, 0.1):
                continue
            yaw = math.atan2(ty - y, tx - x) + rng.uniform(-0.4, 0.4)
            T = camera_transform(x, y, yaw, rng.uniform(0.0, 0.45))
            img = renderer.render(T, K, args.width, args.height, noise=rng.uniform(0, 6))
            img = cv2.convertScaleAbs(img, alpha=rng.uniform(0.7, 1.3), beta=rng.uniform(-30, 30))
            labels = []
            for cls, pts in ((0, cube_corners(world.cube)), (1, board_corners(world.goal))):
                box = project_bbox(pts, T, K, args.width, args.height)
                if box is not None:
                    labels.append(f'{cls} ' + ' '.join(f'{v:.6f}' for v in box))
            split = 'val' if rng.random() < args.val else 'train'
            name = f'sim_{written:05d}'
            cv2.imwrite(os.path.join(args.out, 'images', split, name + '.jpg'), img)
            with open(os.path.join(args.out, 'labels', split, name + '.txt'), 'w') as f:
                f.write('\n'.join(labels) + ('\n' if labels else ''))
            written += 1
            if written >= args.images:
                break
    with open(os.path.join(args.out, 'data.yaml'), 'w') as f:
        f.write(f'path: {os.path.abspath(args.out)}\ntrain: images/train\nval: images/val\n'
                f'names:\n' + ''.join(f'  {i}: {n}\n' for i, n in enumerate(CLASSES)))
    print(f'{written} images in {args.out}')


if __name__ == '__main__':
    main()
