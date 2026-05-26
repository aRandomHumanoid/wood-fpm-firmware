from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import MotionController


def build_motion(probe_surface_x=None):
    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=True,
        rapid_feed_mm_s=15.0,
        scan_feed_mm_s=2.0,
    )
    motion.marlin._probe_surface_x = probe_surface_x  # noqa: SLF001
    return motion


def test_probe_to_x_reports_contact_position():
    motion = build_motion(lambda y: 2.5 + abs(y) * 0.1)
    try:
        motion.move_to(10.0, 4.0, wait=True)
        motion.set_feed_mm_s(motion.scan_feed_mm_s)

        hit = motion.probe_to_x(0.0, 4.0)

        assert hit is True
        assert abs(motion.state.x - 2.9) < 1e-6
        assert abs(motion.state.y - 4.0) < 1e-6
    finally:
        motion.shutdown()


def test_probe_to_x_returns_false_on_miss():
    motion = build_motion(lambda _y: None)
    try:
        motion.move_to(10.0, -3.0, wait=True)
        motion.set_feed_mm_s(motion.scan_feed_mm_s)

        hit = motion.probe_to_x(0.0, -3.0)

        assert hit is False
        assert abs(motion.state.x - 0.0) < 1e-6
        assert abs(motion.state.y - -3.0) < 1e-6
    finally:
        motion.shutdown()