"""ScanRunner against mock motion.

The production scan loop now uses a blocking Marlin G38.2 probe move toward
the configured X target. These tests keep ScanRunner isolated and verify the
scan sequencing against a duck-typed motion object.
"""

import threading

from src.core.limits import MotionLimits
from src.core.scan import ScanRequest, ScanRunner


class FakeState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0


class FakeMotion:
    """Single-threaded motion sim with an X-directed probe stroke."""

    def __init__(self, contact_fn):
        self.contact_fn = contact_fn
        self.state = FakeState()
        self.rapid_feed_mm_s = 15.0
        self.scan_feed_mm_s = 2.0
        self.limits = MotionLimits(v_max_mm_s=20, a_max_mm_s2=100)
        self.feed_history = []
        self.halt_calls = 0

    def set_feed_mm_s(self, v):
        self.feed_history.append(v)

    def halt(self):
        self.halt_calls += 1

    def move_to(self, x, y, wait=True, **_):
        self.state.x = x
        self.state.y = y

    def probe_to_x(self, target_x, y, **_):
        self.state.y = y
        contact_x = self.contact_fn(y)
        lo, hi = sorted((target_x, self.state.x))
        if contact_x is None or not (lo <= contact_x <= hi):
            self.state.x = target_x
            return False
        self.state.x = contact_x
        return True


def test_scan_traces_known_contour():
    # A V-shape expressed as x(y), probed from x=10 toward x=0.
    def contour(y):
        return abs(y)

    motion = FakeMotion(contour)
    runner = ScanRunner(motion=motion)
    received = []

    started = threading.Event()
    done = threading.Event()
    runner.on_started = lambda req: started.set()
    runner.on_point = lambda pt: received.append(pt)
    runner.on_complete = lambda sid: done.set()

    req = ScanRequest(x_max=10, probe_target_x=0, y_max=10, y_min=-10, n_samples=5, scan_id="t1")

    runner.start(req)
    assert started.wait(timeout=2.0)
    assert done.wait(timeout=30.0)

    assert len(received) == 5
    # Every sampled Y should produce a contact within ~step tolerance.
    for pt in received:
        expected = contour(pt.y)
        assert pt.x is not None, f"missed contact at y={pt.y}"
        assert abs(pt.x - expected) < 1.0, f"y={pt.y} x={pt.x} exp={expected}"


def test_scan_records_no_contact_when_surface_absent():
    def contour(_y):
        return None     # no surface anywhere

    motion = FakeMotion(contour)
    runner = ScanRunner(motion)
    pts = []
    done = threading.Event()
    runner.on_point = lambda p: pts.append(p)
    runner.on_complete = lambda _sid: done.set()

    runner.start(ScanRequest(x_max=4, probe_target_x=0, y_max=5, y_min=-5, n_samples=3, scan_id="t2"))
    assert done.wait(timeout=20.0)
    assert len(pts) == 3
    assert all(p.x is None for p in pts)


def test_scan_uses_requested_probe_target_x():
    def contour(_y):
        return -1.0

    motion = FakeMotion(contour)
    runner = ScanRunner(motion)
    pts = []
    done = threading.Event()
    runner.on_point = lambda p: pts.append(p)
    runner.on_complete = lambda _sid: done.set()

    runner.start(ScanRequest(x_max=4, probe_target_x=-2, y_max=5, y_min=-5, n_samples=3, scan_id="t3"))
    assert done.wait(timeout=20.0)
    assert len(pts) == 3
    assert all(p.x == -1.0 for p in pts)


def test_scan_uses_requested_probe_speed():
    motion = FakeMotion(lambda _y: 1.0)
    runner = ScanRunner(motion)
    done = threading.Event()
    runner.on_complete = lambda _sid: done.set()

    runner.start(
        ScanRequest(
            x_max=4,
            probe_target_x=0,
            y_max=5,
            y_min=-5,
            n_samples=2,
            probe_speed_mm_s=3.5,
            scan_id="t4",
        )
    )
    assert done.wait(timeout=20.0)
    assert 3.5 in motion.feed_history


def test_scan_abort_does_not_halt_motion():
    motion = FakeMotion(lambda _y: 1.0)
    runner = ScanRunner(motion)

    runner.abort()

    assert runner._abort.is_set() is True  # noqa: SLF001
    assert motion.halt_calls == 0

