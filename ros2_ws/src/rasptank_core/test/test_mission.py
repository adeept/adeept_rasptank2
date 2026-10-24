"""The mission state machine against a minimal kinematic world."""
import math

from rasptank_core.kinematics import Odometry
from rasptank_core.mission import (APROXIMAR_CUBO, BUSCAR_META, COGER, COMPROBAR, FALLO, FIN,
                                   Mission, MissionInput, MissionParams, Target)

DT = 0.05


class World:
    def __init__(self, cube=(0.8, 0.3), goal=(-0.4, 0.9), fail_grasps=0):
        self.odo = Odometry()
        self.cube = list(cube)
        self.goal = goal
        self.held = False
        self.arm_pose = ''
        self.arm_since = 0.0
        self.requested = None
        self.fail_grasps = fail_grasps
        self.reach = MissionParams().grasp_reach

    def to_base(self, x, y):
        p = self.odo.pose
        dx, dy = x - p.x, y - p.y
        c, s = math.cos(-p.yaw), math.sin(-p.yaw)
        return c * dx - s * dy, s * dx + c * dy

    def seen(self, x, y, now, min_dist=0.2):
        bx, by = self.to_base(x, y)
        if bx > min_dist and abs(math.atan2(by, bx)) < math.radians(30) and math.hypot(bx, by) < 2:
            return Target(bx, by, now)
        return None

    def gripper(self):
        p = self.odo.pose
        return p.x + self.reach * math.cos(p.yaw), p.y + self.reach * math.sin(p.yaw)

    def step(self, out, now):
        if out.arm_pose != self.requested:
            self.requested = out.arm_pose
            self.arm_since = now
            if out.arm_pose == 'cerrar':
                gx, gy = self.gripper()
                if math.hypot(gx - self.cube[0], gy - self.cube[1]) < 0.03:
                    if self.fail_grasps:
                        self.fail_grasps -= 1
                    else:
                        self.held = True
            if out.arm_pose == 'abrir' and self.held:
                self.held = False
                self.cube = list(self.gripper())
        if self.requested and now - self.arm_since > 0.5:
            self.arm_pose = self.requested
        self.odo.update(out.v, out.w, DT)
        if self.held:
            self.cube = list(self.gripper())

    def inputs(self, now):
        cube = None if self.held else self.seen(*self.cube, now, min_dist=0.25)
        moving = self.requested is not None and self.arm_pose != self.requested
        return MissionInput(now, self.odo.pose, cube, self.seen(*self.goal, now),
                            self.arm_pose, moving)


def run(world, mission, seconds=200):
    now = 0.0
    mission.start()
    while now < seconds and not mission.finished:
        out = mission.step(world.inputs(now))
        world.step(out, now)
        now += DT
    return now


def test_full_mission_succeeds():
    world, mission = World(), Mission()
    run(world, mission)
    assert mission.state == FIN, (mission.state, mission.info, mission.history)
    gx, gy = world.goal
    assert math.hypot(world.cube[0] - gx, world.cube[1] - gy) < 0.35
    assert not world.held
    assert mission.history.index(COGER) < mission.history.index(BUSCAR_META)


def test_failed_grasp_is_retried():
    world, mission = World(fail_grasps=1), Mission()
    run(world, mission)
    assert mission.state == FIN, (mission.state, mission.info, mission.history)
    assert mission.retries == 1
    assert mission.history.count(APROXIMAR_CUBO) >= 2
    assert COMPROBAR in mission.history


def test_gives_up_after_max_retries():
    world, mission = World(fail_grasps=10), Mission(MissionParams(max_retries=1))
    run(world, mission)
    assert mission.state == FALLO and 'coger' in mission.info


def test_no_cube_fails_after_search_timeout():
    world, mission = World(cube=(50.0, 50.0)), Mission(MissionParams(search_timeout=10))
    run(world, mission)
    assert mission.state == FALLO and 'cubo' in mission.info


def test_stop_returns_to_wait():
    world, mission = World(), Mission()
    mission.start()
    mission.step(world.inputs(0.0))
    mission.stop()
    out = mission.step(world.inputs(0.1))
    assert (out.v, out.w, out.state) == (0.0, 0.0, 'ESPERA')
