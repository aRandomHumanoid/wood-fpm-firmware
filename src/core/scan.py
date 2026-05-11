"""1-D contour scan orchestration.

For each X sample (evenly spaced across [x_min, x_max]):
  1. Rapid to (x, y_max).
  2. Slow-feed toward (x, y_min), watching the probe.
  3. On probe trigger: halt, record (x, current_y) as a contact point.
     If no trigger before reaching y_min: record (x, None).
  4. Retract rapid to (x, y_max).
  5. Advance X.
Emits scan_started / scan_point / scan_complete via callbacks.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, asdict
from typing import Callable, Optional

import numpy as np

from .motion import MotionController
from .probe import ProbeMonitor

log = logging.getLogger(__name__)


@dataclass
class ScanRequest:
    x_min: float
    x_max: float
    y_max: float
    y_min: float
    n_samples: int
    scan_id: str = ""

    def to_dict(self):
        return asdict(self)


@dataclass
class ScanPoint:
    scan_id: str
    index: int
    x: float
    y: Optional[float]   # None == no contact

    def to_dict(self):
        return {"scan_id": self.scan_id, "index": self.index, "x": self.x, "y": self.y}


PointCb = Callable[[ScanPoint], None]
StartedCb = Callable[[ScanRequest], None]
CompleteCb = Callable[[str], None]   # scan_id


class ScanRunner:
    def __init__(self, motion: MotionController, probe: ProbeMonitor):
        self.motion = motion
        self.probe = probe
        self._thread: Optional[threading.Thread] = None
        self._abort = threading.Event()
        self._running = False

        self.on_started: Optional[StartedCb] = None
        self.on_point: Optional[PointCb] = None
        self.on_complete: Optional[CompleteCb] = None

    @property
    def running(self) -> bool:
        return self._running

    def start(self, req: ScanRequest):
        if self._running:
            raise RuntimeError("scan already running")
        self._abort.clear()
        self._thread = threading.Thread(target=self._run, args=(req,), daemon=True)
        self._thread.start()

    def abort(self):
        self._abort.set()
        self.motion.halt()

    def _run(self, req: ScanRequest):
        self._running = True
        if self.on_started:
            try: self.on_started(req)
            except Exception: log.exception("on_started")

        xs = np.linspace(req.x_min, req.x_max, max(req.n_samples, 1))
        try:
            for i, x in enumerate(xs):
                if self._abort.is_set():
                    break
                # 1. Rapid to top.
                self.motion.set_feed_mm_s(self.motion.rapid_feed_mm_s)
                self.motion.move_to(float(x), req.y_max, wait=True)
                if self._abort.is_set():
                    break

                # 2. Slow descent toward y_min, with probe watch.
                self.motion.set_feed_mm_s(self.motion.scan_feed_mm_s)
                triggered = threading.Event()

                def on_change(state: bool, _evt=triggered):
                    if state:
                        _evt.set()

                self.probe.on_change(on_change)
                contact_y: Optional[float] = None
                try:
                    # already in contact before starting?
                    if self.probe.is_triggered():
                        contact_y = self.motion.state.y
                    else:
                        self.motion.move_to(float(x), req.y_min, wait=False)
                        while True:
                            if self._abort.is_set():
                                break
                            if triggered.is_set():
                                self.motion.halt()
                                contact_y = self.motion.state.y
                                break
                            if not self.motion.is_busy():
                                # reached y_min without contact
                                break
                            time.sleep(0.002)
                finally:
                    try:
                        self.probe._listeners.remove(on_change)  # noqa: SLF001
                    except ValueError:
                        pass

                pt = ScanPoint(
                    scan_id=req.scan_id,
                    index=i,
                    x=float(x),
                    y=contact_y,
                )
                if self.on_point:
                    try: self.on_point(pt)
                    except Exception: log.exception("on_point")

                if self._abort.is_set():
                    break

                # 3. Retract.
                self.motion.set_feed_mm_s(self.motion.rapid_feed_mm_s)
                self.motion.move_to(float(x), req.y_max, wait=True)
        except Exception:
            log.exception("scan run")
        finally:
            # restore default feed
            self.motion.set_feed_mm_s(self.motion.limits.v_max_mm_s)
            self._running = False
            if self.on_complete:
                try: self.on_complete(req.scan_id)
                except Exception: log.exception("on_complete")
