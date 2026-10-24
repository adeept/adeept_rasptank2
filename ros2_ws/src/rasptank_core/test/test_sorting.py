"""Sorter alone, and the whole sorting loop against a kinematic world."""
import math

from rasptank_core.kinematics import Odometry, Pose2D
from rasptank_core.mission import (APROXIMAR_CUBO, BUSCAR_CUBO, FALLO, FIN, Mission, MissionInput,
                                   MissionParams)
from rasptank_core.sorting import Sorter

GOALS = {'cubo_rojo': 0, 'cubo_azul': 1}


def test_nearest_cube_lock_and_goal():
    s = Sorter(GOALS)
    pose = Pose2D()
    s.observe([('cubo_rojo', 0.8, 0.0), ('cubo_azul', 0.5, 0.2), ('meta_1', 1.5, -0.5)], pose, 0.0)
    t = s.cube_target(pose, 0.1)
    assert (round(t.x, 3), round(t.y, 3)) == (0.5, 0.2)             # the nearest
    s.lock()
    assert s.locked_class == 'cubo_azul'
    # the robot moved 0.2 m forward: same cubes, new robot frame
    pose = Pose2D(0.2, 0.0, 0.0)
    s.observe([('cubo_rojo', 0.6, 0.0), ('cubo_azul', 0.3, 0.2), ('meta_1', 1.3, -0.5)], pose, 1.0)
    t = s.cube_target(pose, 1.0)
    assert (round(t.x, 3), round(t.y, 3)) == (0.3, 0.2)             # still the locked one
    g = s.goal_target(pose, 1.0)
    assert (round(g.x, 3), round(g.y, 3)) == (1.3, -0.5)
    # once picked up, another cube of the same class elsewhere is NOT it
    s.observe([('cubo_azul', 0.3, -0.5)], pose, 2.0)
    assert s.cube_target(pose, 2.0) is None


def test_cubes_already_on_a_goal_are_ignored():
    s = Sorter(GOALS)
    pose = Pose2D()
    s.observe([('meta_0', 1.0, 0.0), ('cubo_rojo', 0.9, 0.05), ('cubo_azul', 0.6, 0.5)], pose, 0.0)
    t = s.cube_target(pose, 0.0)
    assert round(t.y, 3) == 0.5                                     # not the one at the goal


def test_stale_detections_do_not_count():
    s = Sorter(GOALS, max_age=0.5)
    s.observe([('cubo_rojo', 0.8, 0.0)], Pose2D(), 0.0)
    assert s.cube_target(Pose2D(), 1.0) is None


# ------------------------------------------------------------ full loop

class SortWorld:
    """Point robot with a 60 deg camera, several cubes and goals."""

    REACH = MissionParams().grasp_reach

    def __init__(self):
        self.odo = Odometry()
        self.cubes = {'r1': ['cubo_rojo', 0.8, 0.3], 'r2': ['cubo_rojo', 0.4, -0.6],
                      'a1': ['cubo_azul', 0.9, -0.3], 'a2': ['cubo_azul', 0.2, 0.7]}
        self.goals = {0: (1.6, 1.0), 1: (1.6, -1.0)}
        self.held = None
        self.requested = None
        self.arm_pose = ''
        self.since = 0.0

    def to_base(self, x, y):
        p = self.odo.pose
        dx, dy = x - p.x, y - p.y
        c, s = math.cos(-p.yaw), math.sin(-p.yaw)
        return c * dx - s * dy, s * dx + c * dy

    def visible(self, x, y, min_d=0.25):
        bx, by = self.to_base(x, y)
        ok = bx > min_d and abs(math.atan2(by, bx)) < math.radians(30) and math.hypot(bx, by) < 2.5
        return (bx, by) if ok else None

    def detections(self):
        out = []
        for name, (cls, x, y) in self.cubes.items():
            if name != self.held and (v := self.visible(x, y)):
                out.append((cls, *v))
        for gid, (x, y) in self.goals.items():
            if v := self.visible(x, y, 0.1):
                out.append((f'meta_{gid}', *v))
        return out

    def gripper(self):
        p = self.odo.pose
        return p.x + self.REACH * math.cos(p.yaw), p.y + self.REACH * math.sin(p.yaw)

    def step(self, out, now, dt):
        if out.arm_pose != self.requested:
            self.requested, self.since = out.arm_pose, now
            gx, gy = self.gripper()
            if out.arm_pose == 'cerrar' and self.held is None:
                for name, (_, x, y) in self.cubes.items():
                    if math.hypot(gx - x, gy - y) < 0.03:
                        self.held = name
            if out.arm_pose == 'abrir' and self.held is not None:
                self.cubes[self.held][1:] = [gx, gy]
                self.held = None
        if self.requested and now - self.since > 0.5:
            self.arm_pose = self.requested
        self.odo.update(out.v, out.w, dt)
        if self.held is not None:
            self.cubes[self.held][1:] = list(self.gripper())

    def sorted_right(self):
        right = 0
        for cls, x, y in self.cubes.values():
            gx, gy = self.goals[GOALS[cls]]
            right += math.hypot(x - gx, y - gy) < 0.4
        return right


def test_sorts_all_cubes():
    world, mission, sorter = SortWorld(), Mission(), Sorter(GOALS)
    dt, now, previous = 0.05, 0.0, None
    mission.start()
    while now < 900 and len(sorter.delivered) < 4:
        pose = world.odo.pose
        sorter.observe(world.detections(), pose, now)
        state = mission.state
        if state == APROXIMAR_CUBO and previous != APROXIMAR_CUBO:
            sorter.lock()
        elif state == BUSCAR_CUBO:
            sorter.unlock()
        previous = state
        inp = MissionInput(now, pose, sorter.cube_target(pose, now), sorter.goal_target(pose, now),
                           world.arm_pose, world.requested is not None
                           and world.arm_pose != world.requested)
        out = mission.step(inp)
        world.step(out, now, dt)
        if mission.state == FIN:
            sorter.deliver()
            mission.start()
        assert mission.state != FALLO, (mission.info, sorter.delivered)
        now += dt
    assert sorted(sorter.delivered) == ['cubo_azul', 'cubo_azul', 'cubo_rojo', 'cubo_rojo']
    assert world.sorted_right() == 4
