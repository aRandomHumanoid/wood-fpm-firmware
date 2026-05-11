"""ScanRunner against mock motion + probe.

We don't exercise the real MotionController here — its threading and EPOS
ctypes plumbing belong in a hardware-in-the-loop test. Instead we feed
ScanRunner a duck-typed pair that simulates motion + a probe contour.
"""

import threading
import time

import pytest

from src.core.limits import MotionLimits
from src.core.scan import ScanRequest, ScanRunner, ScanPoint


class FakeState:
    def __init__(self):
        self.y = 0.0


class FakeMotion:
    """Single-threaded motion sim. wait=True snaps to the target; wait=False
    schedules the descent so that the probe trips (via on_change) at the
    contour y, then halts. All state mutations happen on a single timer."""

    def __init__(self, contact_fn):
        self.contact_fn = contact_fn
        self.state = FakeState()
        self.rapid_feed_mm_s = 15.0
        self.scan_feed_mm_s = 2.0
        self.limits = MotionLimits(v_max_mm_s=20, a_max_mm_s2=100)
        self._busy = False
        self._fake_probe = None
        self._timer: threading.Timer | None = None

    def set_feed_mm_s(self, v):
        pass

    def halt(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self._busy = False

    def is_busy(self) -> bool:
        return self._busy

    def move_to(self, x, y, wait=True, **_):
        # Any new move cancels an in-flight scheduled descent.
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if wait:
            self.state.y = y
            self._busy = False
            return

        # Compute where (if anywhere) the probe would trip during this descent.
        cont = self.contact_fn(x) if self._fake_probe is not None else None
        start_y = self.state.y
        end_y = y
        if cont is not None and (start_y >= cont >= end_y):
            # Trip at y = cont after a short delay.
            self._busy = True

            def _trip():
                self.state.y = cont
                # _fake_probe could be None in rare races; guarded.
                if self._fake_probe is not None:
                    self._fake_probe._notify(True)

            self._timer = threading.Timer(0.01, _trip)
            self._timer.daemon = True
            self._timer.start()
        else:
            # No contact — just travel to end_y after a delay, then mark idle.
            self._busy = True

            def _arrive():
                self.state.y = end_y
                self._busy = False

            self._timer = threading.Timer(0.02, _arrive)
            self._timer.daemon = True
            self._timer.start()


class FakeProbe:
    def __init__(self):
        self._listeners = []
        self._triggered = False

    def on_change(self, fn):
        self._listeners.append(fn)

    def is_triggered(self):
        return self._triggered

    def _notify(self, t: bool):
        self._triggered = t
        for fn in list(self._listeners):
            fn(t)


def test_scan_traces_known_contour():
    # A V-shape: contact at y = |x| - 5 inside [-10, 10], so the probe trips
    # for any x in that range when descending from y_max=10 toward y_min=-10.
    def contour(x):
        v = abs(x) - 5.0
        return v if v >= -10 else None

    motion = FakeMotion(contour)
    probe = FakeProbe()
    motion._fake_probe = probe

    runner = ScanRunner(motion=motion, probe=probe)
    received = []

    started = threading.Event()
    done = threading.Event()
    runner.on_started = lambda req: started.set()
    runner.on_point = lambda pt: received.append(pt)
    runner.on_complete = lambda sid: done.set()

    req = ScanRequest(x_min=-10, x_max=10, y_max=10, y_min=-10, n_samples=5, scan_id="t1")
    # Reset probe between samples
    def reset_per_step(pt):
        probe._triggered = False
    runner.on_point = lambda pt: (received.append(pt), reset_per_step(pt))

    runner.start(req)
    assert started.wait(timeout=2.0)
    assert done.wait(timeout=30.0)

    assert len(received) == 5
    # Every X should produce a contact within ~step tolerance of the contour.
    for pt in received:
        expected = contour(pt.x)
        assert pt.y is not None, f"missed contact at x={pt.x}"
        assert abs(pt.y - expected) < 1.0, f"x={pt.x} y={pt.y} exp={expected}"


def test_scan_records_no_contact_when_surface_absent():
    def contour(_x):
        return None     # no surface anywhere

    motion = FakeMotion(contour)
    probe = FakeProbe()
    motion._fake_probe = probe

    runner = ScanRunner(motion, probe)
    pts = []
    done = threading.Event()
    runner.on_point = lambda p: pts.append(p)
    runner.on_complete = lambda _sid: done.set()

    runner.start(ScanRequest(x_min=0, x_max=4, y_max=5, y_min=-5, n_samples=3, scan_id="t2"))
    assert done.wait(timeout=20.0)
    assert len(pts) == 3
    assert all(p.y is None for p in pts)
