import pytest

from src.core.limits import MachineBounds, MotionLimits, BoundsError


def test_bounds_contains():
    b = MachineBounds(x_min=-10, x_max=10, y_min=-5, y_max=5)
    assert b.contains(0, 0)
    assert b.contains(-10, 5)
    assert not b.contains(11, 0)
    assert not b.contains(0, -6)


def test_bounds_check_raises():
    b = MachineBounds(x_min=0, x_max=1, y_min=0, y_max=1)
    b.check(0.5, 0.5)
    with pytest.raises(BoundsError):
        b.check(2, 0)


def test_bounds_clamp():
    b = MachineBounds(x_min=0, x_max=10, y_min=-5, y_max=5)
    assert b.clamp(15, 0) == (10, 0)
    assert b.clamp(-5, -10) == (0, -5)


def test_motion_limits_clamp():
    L = MotionLimits(v_max_mm_s=20, a_max_mm_s2=100)
    assert L.clamp_velocity(25) == 20
    assert L.clamp_velocity(-5) == 0
    assert L.clamp_acceleration(50) == 50
