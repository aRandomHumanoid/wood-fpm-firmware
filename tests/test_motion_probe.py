import threading
import time

import pytest

import src.core.motion as motion_module

from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import MotionController
from src.drivers.marlin import MarlinError, Position


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


def test_probe_to_x_uses_probe_reply_when_position_query_is_stale():
    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=False,
        auto_connect=False,
        scan_feed_mm_s=2.0,
    )

    class FakeMarlin:
        def __init__(self):
            self.position_calls = 0

        def probe_target(self, axes, feedrate_mm_min=None, timeout_s=300.0):
            assert axes == {"X": 0.0, "Y": 4.0}
            assert feedrate_mm_min == 120.0
            assert timeout_s == 120.0
            return {"X": 2.9, "Y": 4.0, "Z": 0.0}

        def position(self):
            self.position_calls += 1
            return Position(
                logical={"X": 0.0, "Y": 4.0, "Z": 0.0},
                counts={"X": 0, "Y": 400, "Z": 0},
            )

        def disable_steppers(self):
            pass

        def close(self):
            pass

    fake_marlin = FakeMarlin()
    motion.marlin = fake_marlin
    motion._set_state(x=10.0, y=4.0, z=0.0)  # noqa: SLF001

    try:
        motion.set_feed_mm_s(motion.scan_feed_mm_s)

        hit = motion.probe_to_x(0.0, 4.0)

        assert hit is True
        assert motion.state.x == 2.9
        assert motion.state.y == 4.0
        assert motion.state.z == 0.0
        assert fake_marlin.position_calls == 1
    finally:
        motion.shutdown()


def test_probe_to_x_suspends_background_polling_until_probe_completes(monkeypatch):
    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=False,
        auto_connect=False,
        scan_feed_mm_s=2.0,
    )

    started = threading.Event()
    release = threading.Event()
    refresh_calls = []

    class FakeMarlin:
        def probe_target(self, axes, feedrate_mm_min=None, timeout_s=300.0):
            assert axes == {"X": 0.0, "Y": 4.0}
            assert feedrate_mm_min == 120.0
            assert timeout_s == 120.0
            started.set()
            assert release.wait(timeout=2.0)
            return {"X": 2.9, "Y": 4.0, "Z": 0.0}

        def disable_steppers(self):
            pass

        def close(self):
            pass

    motion.marlin = FakeMarlin()

    def fake_refresh_position():
        refresh_calls.append(time.monotonic())
        return True

    monkeypatch.setattr(motion, "_refresh_position", fake_refresh_position)

    try:
        motion.set_feed_mm_s(motion.scan_feed_mm_s)
        motion.start_polling(hz=50.0)
        probe_thread = threading.Thread(target=motion.probe_to_x, args=(0.0, 4.0), daemon=True)
        probe_thread.start()

        assert started.wait(timeout=1.0)
        baseline_calls = len(refresh_calls)
        time.sleep(0.12)
        assert len(refresh_calls) == baseline_calls

        release.set()
        probe_thread.join(timeout=1.0)
        assert probe_thread.is_alive() is False
        assert len(refresh_calls) >= baseline_calls + 1
    finally:
        motion.shutdown()


def test_connect_serial_refreshes_position_from_new_driver(monkeypatch):
    motion = build_motion()

    class FakeMarlin:
        def set_absolute_mode(self):
            pass

        def position(self):
            return Position(
                logical={"X": 3.0, "Y": 4.0, "Z": 0.0},
                counts={"X": 300, "Y": 400, "Z": 0},
            )

        def close(self):
            pass

    try:
        motion.move_to(10.0, -2.0, wait=True)

        monkeypatch.setattr(motion_module, "make_marlin", lambda simulate, **kwargs: FakeMarlin())

        motion.connect_serial(port="/dev/ttyUSB0", baudrate=115200, simulate=False)

        assert motion.state.x == 3.0
        assert motion.state.y == 4.0
        assert motion.state.z == 0.0
        assert motion.serial_status()["simulate"] is False
    finally:
        motion.shutdown()


def test_motion_can_start_disconnected_for_ui_managed_serial(monkeypatch):
    calls = []

    def fake_make_marlin(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("unexpected startup connection")

    monkeypatch.setattr(motion_module, "make_marlin", fake_make_marlin)

    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=False,
        auto_connect=False,
    )
    try:
        assert calls == []
        status = motion.serial_status()
        assert status["connected"] is False
        assert status["simulate"] is False
        assert status["last_error"] == ""
    finally:
        motion.shutdown()


def test_unhomed_jog_uses_relative_move_without_absolute_bounds():
    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=False,
        auto_connect=False,
    )

    class FakeMarlin:
        def __init__(self):
            self.relative_moves = []

        def position(self):
            return Position(
                logical={"X": 12.0, "Y": 0.0, "Z": 0.0},
                counts={"X": 1200, "Y": 0, "Z": 0},
            )

        def move(self, *_args, **_kwargs):
            raise AssertionError("absolute move should not be used while unhomed")

        def move_relative(self, axes, feedrate_mm_min=None):
            self.relative_moves.append((axes, feedrate_mm_min))

        def disable_steppers(self):
            pass

        def close(self):
            pass

    fake_marlin = FakeMarlin()
    motion.marlin = fake_marlin

    try:
        motion.jog(-1.0, 0.0, wait=False)

        assert fake_marlin.relative_moves == [(
            {motion._x_letter: -1.0, motion._y_letter: 0.0},
            motion.limits.v_max_mm_s * 60.0,
        )]
    finally:
        motion.shutdown()


def test_unhomed_jog_timeout_clears_busy_tracking():
    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=False,
        auto_connect=False,
    )

    class StalledMarlin:
        def position(self):
            return Position(
                logical={"X": 0.0, "Y": 0.0, "Z": 0.0},
                counts={"X": 0, "Y": 0, "Z": 0},
            )

        def move_relative(self, _axes, feedrate_mm_min=None):
            assert feedrate_mm_min is not None

        def disable_steppers(self):
            pass

        def close(self):
            pass

    motion.marlin = StalledMarlin()

    try:
        with pytest.raises(MarlinError, match="timeout waiting for move to complete"):
            motion.jog(1.0, 0.0, wait=True, timeout_s=0.01)

        assert motion.state.busy is False
        assert motion.state.fault is True
        assert motion._busy_flag is False  # noqa: SLF001
        assert motion._target is None  # noqa: SLF001

        motion._refresh_position()  # noqa: SLF001
        assert motion.state.busy is False
    finally:
        motion.shutdown()


def test_move_to_wait_uses_blocking_motion_completion_with_single_refresh():
    motion = MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        simulate=False,
        auto_connect=False,
    )

    class FakeMarlin:
        def __init__(self):
            self.calls = []

        def move(self, axes, feedrate_mm_min=None):
            self.calls.append(("move", axes, feedrate_mm_min))

        def wait_for_motion(self, timeout_s=60.0):
            self.calls.append(("wait_for_motion", timeout_s))

        def position(self):
            self.calls.append(("position",))
            return Position(
                logical={"X": 6.0, "Y": 2.0, "Z": 0.0},
                counts={"X": 600, "Y": 200, "Z": 0},
            )

        def disable_steppers(self):
            pass

        def close(self):
            pass

    fake_marlin = FakeMarlin()
    motion.marlin = fake_marlin

    try:
        motion.move_to(6.0, 2.0, wait=True)

        assert fake_marlin.calls == [
            ("move", {"X": 6.0, "Y": 2.0}, motion.limits.v_max_mm_s * 60.0),
            ("wait_for_motion", 60.0),
            ("position",),
        ]
        assert motion.state.x == 6.0
        assert motion.state.y == 2.0
        assert motion.state.busy is False
    finally:
        motion.shutdown()


def test_motion_controller_rejects_invalid_feedrate_values():
    with pytest.raises(ValueError, match="rapid_feed_mm_s"):
        MotionController(
            bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
            limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
            simulate=False,
            auto_connect=False,
            rapid_feed_mm_s="3-.0",
        )