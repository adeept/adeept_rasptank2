from rasptank_core.kinematics import Odometry
from rasptank_core.route import RouteFollower


def test_follow_square():
    route = RouteFollower()
    route.set_route([(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)])
    odo = Odometry()
    for _ in range(3000):
        v, w = route.step(odo.pose)
        odo.update(v, w, 0.02)
        if route.done:
            break
    assert route.done
    assert abs(odo.pose.x) < 0.06 and abs(odo.pose.y) < 0.06


def test_empty_route_stays():
    assert RouteFollower().step(Odometry().pose) == (0.0, 0.0)
