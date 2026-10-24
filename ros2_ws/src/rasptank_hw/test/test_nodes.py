"""base_node and arm_node with the fake driver: safety and poses."""
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from rasptank_hw.arm_node import ArmNode
from rasptank_hw.base_node import BaseNode

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_for(executor, seconds, every=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if every:
            every()
        executor.spin_once(timeout_sec=0.02)


def test_base_watchdog_limits_and_emergency_stop(ros):
    base = BaseNode()
    test = Node('test_base')
    ex = SingleThreadedExecutor()
    ex.add_node(base)
    ex.add_node(test)
    cmd_pub = test.create_publisher(Twist, 'cmd_vel', 10)
    stop_pub = test.create_publisher(Bool, 'emergency_stop', LATCHED)

    fast = Twist()
    fast.linear.x = 5.0
    spin_for(ex, 0.5, lambda: cmd_pub.publish(fast))
    left, right = base.motors.throttle
    assert 0 < left <= 1.0 and left == pytest.approx(right)
    assert base.odom.v == pytest.approx(base.max_linear)       # clamped

    spin_for(ex, 0.8)                                            # no more commands
    assert base.motors.throttle == (0.0, 0.0)                    # watchdog

    stop_pub.publish(Bool(data=True))
    spin_for(ex, 0.5, lambda: cmd_pub.publish(fast))
    assert base.motors.throttle == (0.0, 0.0)
    stop_pub.publish(Bool(data=False))
    spin_for(ex, 0.5, lambda: cmd_pub.publish(fast))
    assert base.motors.throttle[0] > 0
    base.destroy_node()
    test.destroy_node()


def test_arm_poses_and_emergency_stop(ros):
    node = ArmNode(parameter_overrides=[
        Parameter('poses.abrir.gripper_joint', value=0.6),
        Parameter('poses.plegado.shoulder_joint', value=1.2),
        Parameter('max_speed', value=3.0)])
    test = Node('test_arm')
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(test)
    pose_pub = test.create_publisher(String, 'arm/pose', 10)
    stop_pub = test.create_publisher(Bool, 'emergency_stop', LATCHED)
    spin_for(ex, 0.3)
    assert node.arm.position['shoulder_joint'] == pytest.approx(1.2)     # initial pose
    pose_pub.publish(String(data='abrir'))
    spin_for(ex, 0.6)
    assert node.arm.position['gripper_joint'] == pytest.approx(0.6)
    assert node.pose_name == 'abrir' and not node.arm.moving()
    stop_pub.publish(Bool(data=True))
    spin_for(ex, 0.2)
    pose_pub.publish(String(data='plegado'))
    spin_for(ex, 0.2)
    assert node.pose_name == 'abrir'                                        # ignored
    node.destroy_node()
    test.destroy_node()
