import math

from rasptank_core.governor import GovernorParams, forward_limit, govern

P = GovernorParams()


def test_free_road_passes():
    assert govern(0.15, 0.3, 1.5, 0.1, P) == (0.15, 0.3, False)


def test_stops_forward_but_lets_turn_and_reverse():
    assert govern(0.15, 0.3, 0.10, 0.1, P) == (0.0, 0.3, True)
    assert govern(-0.1, 0.0, 0.05, 0.1, P) == (-0.1, 0.0, False)
    assert govern(0.0, 1.0, 0.05, 0.1, P) == (0.0, 1.0, False)


def test_slows_down_progressively():
    a = forward_limit(0.20, P)
    b = forward_limit(0.35, P)
    assert 0 < a < b < 0.2
    assert forward_limit(P.stop_distance + 1e-6, P) >= P.min_speed


def test_fail_closed_without_data():
    assert govern(0.1, 0.0, None, None, P)[0] == 0.0
    assert govern(0.1, 0.0, 2.0, 5.0, P)[0] == 0.0            # stale
    assert govern(0.1, 0.0, math.nan, 0.1, P)[0] == 0.0
