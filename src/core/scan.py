"""1-D contour scan orchestration.

For each Y sample (evenly spaced across [y_min, y_max]):
    1. Rapid to (x_max, y).
    2. Probe at probe_speed_mm_s toward probe_target_x with Marlin G38.2.
    3. On probe trigger: record (current_x, y) as a contact point.
         If probe_target_x is reached without contact: record (None, y).
    4. Retract rapid to (x_max, y).
    5. Advance Y.
Emits scan_started / scan_point / scan_complete via callbacks.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, asdict
from typing import Callable, Optional

import numpy as np

from .motion import MotionController

log = logging.getLogger(__name__)

DEFAULT_PROBE_TARGET_X = 0.0


@dataclass
class ScanRequest:
    x_max: float
    y_max: float
    y_min: float
    n_samples: int
    probe_target_x: float = DEFAULT_PROBE_TARGET_X
    probe_speed_mm_s: float = 2.0
    scan_id: str = ""

    def to_dict(self):
        return asdict(self)


@dataclass
class ScanPoint:
    scan_id: str
    index: int
    x: Optional[float]   # None == no contact
    y: Optional[float]

    def to_dict(self):
        return {"scan_id": self.scan_id, "index": self.index, "x": self.x, "y": self.y}


PointCb = Callable[[ScanPoint], None]
StartedCb = Callable[[ScanRequest], None]
CompleteCb = Callable[[str], None]   # scan_id


class ScanRunner:
    def __init__(self, motion: MotionController):
        self.motion = motion
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

    def _run(self, req: ScanRequest):
        self._running = True
        if self.on_started:
            try: self.on_started(req)
            except Exception: log.exception("on_started")

        ys = np.linspace(req.y_min, req.y_max, max(req.n_samples, 1))
        try:
            for i, y in enumerate(ys):
                if self._abort.is_set():
                    break
                # 1. Rapid to the start of the probe stroke.
                self.motion.set_feed_mm_s(self.motion.rapid_feed_mm_s)
                self.motion.move_to(req.x_max, float(y), wait=True)
                if self._abort.is_set():
                    break

                # 2. Probe toward the requested X target with Marlin-native G38.2.
                self.motion.set_feed_mm_s(req.probe_speed_mm_s)
                hit = self.motion.probe_to_x(req.probe_target_x, float(y))
                contact_x = self.motion.state.x if hit else None

                pt = ScanPoint(
                    scan_id=req.scan_id,
                    index=i,
                    x=contact_x,
                    y=float(y),
                )
                if self.on_point:
                    try: self.on_point(pt)
                    except Exception: log.exception("on_point")

                if self._abort.is_set():
                    break

                # 3. Retract.
                self.motion.set_feed_mm_s(self.motion.rapid_feed_mm_s)
                self.motion.move_to(req.x_max, float(y), wait=True)
        except Exception:
            log.exception("scan run")
        finally:
            # restore default feed
            self.motion.set_feed_mm_s(self.motion.limits.v_max_mm_s)
            self._running = False
            if self.on_complete:
                try: self.on_complete(req.scan_id)
                except Exception: log.exception("on_complete")
