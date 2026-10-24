import math

import pytest

from rasptank_core.arm import DEFAULT_SERVOS, JOINTS, ArmGeometry, ArmModel, ServoMap, parse_poses


def test_servo_map():
    s = ServoMap(0, center_deg=90, sign=-1, min_deg=20, max_deg=160)
    assert s.to_deg(0.0) == 90
    assert s.to_deg(math.radians(30)) == pytest.approx(60)
    assert s.to_deg(math.radians(-200)) == 160      # clamped
    assert s.to_angle(60) == pytest.approx(math.radians(30))
    low, high = s.limits()
    assert low == pytest.approx(math.radians(-70)) and high == pytest.approx(math.radians(70))


def test_arm_model_slews_and_clamps():
    arm = ArmModel(DEFAULT_SERVOS, max_speed=1.0)
    arm.set_target({'shoulder_joint': 0.5, 'gripper_joint': 10.0})
    assert arm.target['gripper_joint'] == pytest.approx(math.pi / 2)
    arm.step(0.1)
    assert arm.position['shoulder_joint'] == pytest.approx(0.1)
    assert arm.moving()
    for _ in range(30):
        arm.step(0.1)
    assert not arm.moving()
    arm.set_target({'shoulder_joint': -0.5})
    arm.step(0.1)
    arm.hold()
    assert not arm.moving()
    with pytest.raises(ValueError):
        arm.set_target({'nope': 1})


def test_parse_poses():
    poses = parse_poses({'abrir.gripper_joint': 0.6, 'buscar.shoulder_joint': 1.0,
                         'buscar.elbow_joint': -0.5})
    assert poses == {'abrir': {'gripper_joint': 0.6},
                     'buscar': {'shoulder_joint': 1.0, 'elbow_joint': -0.5}}
    with pytest.raises(ValueError):
        parse_poses({'x.bad_joint': 0})


def test_lowered_gripper_reaches_the_floor():
    # pre_coger in rasptank_bringup/config/robot.yaml
    x, z = ArmGeometry().gripper_center(0.0, -0.98)
    assert x == pytest.approx(0.165, abs=0.01)
    assert 0.01 < z < 0.04


def test_joint_order():
    assert JOINTS[3] == 'gripper_joint' and len(JOINTS) == 5
