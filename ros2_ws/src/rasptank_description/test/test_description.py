"""The URDF and the arm geometry used by the mission agree."""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'rasptank_core'))

from rasptank_core.arm import JOINTS, ArmGeometry  # noqa: E402

XACRO = os.path.join(HERE, '..', 'urdf', 'rasptank.urdf.xacro')


def props():
    text = open(XACRO).read()
    return {m[0]: float(m[1]) for m in
            re.findall(r'<xacro:property name="(\w+)" value="([-0-9.]+)"', text)}, text


def test_arm_geometry_matches_urdf():
    p, _ = props()
    g = ArmGeometry()
    assert g.shoulder_x == p['shoulder_x']
    assert g.shoulder_z == round(p['base_height'] + p['shoulder_z'], 6)
    assert g.upper_arm == p['upper_arm']
    assert g.reach == round(p['forearm'] + p['gripper_center'], 6)


def test_every_servo_joint_is_in_the_urdf():
    _, text = props()
    for joint in JOINTS:
        assert f'<joint name="{joint}" type="revolute">' in text
