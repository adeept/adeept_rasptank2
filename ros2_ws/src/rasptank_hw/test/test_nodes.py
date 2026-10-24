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
    base = BaseNode(parameter_overrides=[Parameter('calibration_file', value='')])
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


def test_motor_watchdog_zeroes_when_heartbeat_stops(ros):
    from std_msgs.msg import UInt32
    from rasptank_hw.motor_watchdog_node import MotorWatchdogNode

    dog = MotorWatchdogNode(parameter_overrides=[Parameter('repeat', value=0.1),
                                                 Parameter('timeout', value=0.3)])
    test = Node('test_dog')
    ex = SingleThreadedExecutor()
    ex.add_node(dog)
    ex.add_node(test)
    beat = test.create_publisher(UInt32, 'base/heartbeat', 10)
    spin_for(ex, 0.3)
    assert dog.killer.zeroed > 0                       # nothing alive yet: keeps motors off
    spin_for(ex, 0.2, lambda: beat.publish(UInt32(data=1)))
    before = dog.killer.zeroed
    spin_for(ex, 0.5, lambda: beat.publish(UInt32(data=1)))
    assert dog.killer.zeroed == before                 # heartbeat alive: hands off
    spin_for(ex, 0.8)                                  # base_node "dies"
    assert dog.killer.zeroed > before and dog.tripped
    dog.destroy_node()
    test.destroy_node()


def test_base_calibration_live_save_and_reset(ros, tmp_path):
    from rclpy.parameter import Parameter as P
    from std_srvs.srv import Trigger

    calib = tmp_path / 'calib.yaml'
    base = BaseNode(parameter_overrides=[P('calibration_file', value=str(calib)),
                                         P('max_track_speed', value=0.25)])
    test = Node('test_calib')
    ex = SingleThreadedExecutor()
    ex.add_node(base)
    ex.add_node(test)
    # live change, with validation
    assert base.set_parameters([P('left_gain', value=1.1)])[0].successful
    assert not base.set_parameters([P('left_gain', value=5.0)])[0].successful
    assert base.set_parameters([P('max_track_speed', value=0.2)])[0].successful

    def call(name):
        cli = test.create_client(Trigger, name)
        assert cli.wait_for_service(timeout_sec=2.0)
        fut = cli.call_async(Trigger.Request())
        ex.spin_until_future_complete(fut, timeout_sec=2.0)
        return fut.result()

    assert call('base/save_calibration').success and calib.exists()
    base.destroy_node()
    # a new base_node loads it over robot.yaml
    base2 = BaseNode(parameter_overrides=[P('calibration_file', value=str(calib)),
                                          P('max_track_speed', value=0.25)])
    ex.add_node(base2)
    assert base2.param('max_track_speed') == pytest.approx(0.2)
    assert base2.param('left_gain') == pytest.approx(1.1)
    assert call('base/reset_calibration').success
    assert base2.param('max_track_speed') == pytest.approx(0.25)
    assert not calib.exists()
    base2.destroy_node()
    test.destroy_node()


def test_left_gain_changes_the_left_motor_only(ros):
    from rclpy.parameter import Parameter as P

    base = BaseNode(parameter_overrides=[P('left_gain', value=1.2), P('calibration_file', value='')])
    test = Node('test_gain')
    ex = SingleThreadedExecutor()
    ex.add_node(base)
    ex.add_node(test)
    pub = test.create_publisher(Twist, 'cmd_vel', 10)
    cmd = Twist()
    cmd.linear.x = 0.1
    spin_for(ex, 0.3, lambda: pub.publish(cmd))
    left, right = base.motors.throttle
    assert left == pytest.approx(1.2 * right)
    assert base.odom.v == pytest.approx(0.1)          # odometry: what was asked
    base.destroy_node()
    test.destroy_node()


def test_governor_node_blocks_forward_near_a_wall(ros):
    from sensor_msgs.msg import Range
    from rasptank_hw.governor_node import GovernorNode

    gov = GovernorNode()
    test = Node('test_gov')
    ex = SingleThreadedExecutor()
    ex.add_node(gov)
    ex.add_node(test)
    out = []
    test.create_subscription(Twist, 'cmd_vel_safe', lambda m: out.append(m.linear.x), 10)
    cmd_pub = test.create_publisher(Twist, 'cmd_vel', 10)
    range_pub = test.create_publisher(Range, 'ultrasonic', 10)
    cmd = Twist()
    cmd.linear.x = 0.15

    def tick(distance):
        range_pub.publish(Range(range=distance))
        cmd_pub.publish(cmd)

    spin_for(ex, 0.3, lambda: tick(1.0))
    assert out and out[-1] == pytest.approx(0.15)
    spin_for(ex, 0.3, lambda: tick(0.08))
    assert out[-1] == 0.0
    gov.destroy_node()
    test.destroy_node()


def test_signals_leds_tune_and_emergency(ros):
    from std_msgs.msg import ColorRGBA, String as Str
    from rasptank_hw.signals_node import SignalsNode

    node = SignalsNode(parameter_overrides=[Parameter('led_count', value=3)])
    test = Node('test_signals')
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(test)
    leds = test.create_publisher(ColorRGBA, 'leds', 10)
    buzzer = test.create_publisher(Str, 'buzzer', 10)
    stop = test.create_publisher(Bool, 'emergency_stop', LATCHED)
    spin_for(ex, 0.2)
    leds.publish(ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5))
    spin_for(ex, 0.2)
    assert node.hw.colors == [(0, 127, 0)] * 3
    buzzer.publish(Str(data='A4:0.3'))
    spin_for(ex, 0.15)
    assert node.hw.tone == pytest.approx(440.0)
    spin_for(ex, 0.4)
    assert node.hw.tone is None                       # the tune ended
    buzzer.publish(Str(data='C5:2.0'))
    spin_for(ex, 0.1)
    stop.publish(Bool(data=True))
    spin_for(ex, 0.2)
    assert node.hw.tone is None and node.hw.colors[0] == (255, 0, 0)
    node.destroy_node()
    test.destroy_node()
