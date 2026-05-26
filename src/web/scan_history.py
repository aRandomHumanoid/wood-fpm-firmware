from __future__ import annotations

import csv
import threading
from datetime import datetime, timezone
from pathlib import Path

from ..core.scan import PROBE_TARGET_X, ScanPoint, ScanRequest


class ScanHistoryCsvStore:
    FIELDNAMES = [
        "recorded_at_utc",
        "scan_id",
        "index",
        "hit",
        "x",
        "y",
        "x_max",
        "probe_target_x",
        "y_min",
        "y_max",
        "n_samples",
    ]

    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.Lock()
        self._ensure_file()

    def append_point(self, req: ScanRequest, pt: ScanPoint):
        row = {
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "scan_id": pt.scan_id,
            "index": pt.index,
            "hit": pt.x is not None,
            "x": "" if pt.x is None else f"{pt.x:.6f}",
            "y": "" if pt.y is None else f"{pt.y:.6f}",
            "x_max": f"{req.x_max:.6f}",
            "probe_target_x": f"{PROBE_TARGET_X:.6f}",
            "y_min": f"{req.y_min:.6f}",
            "y_max": f"{req.y_max:.6f}",
            "n_samples": req.n_samples,
        }
        with self._lock:
            self._ensure_file_locked()
            with self.path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=self.FIELDNAMES)
                writer.writerow(row)

    def read_text(self) -> str:
        with self._lock:
            self._ensure_file_locked()
            return self.path.read_text(encoding="utf-8")

    def clear(self):
        with self._lock:
            self._write_header_locked()

    def _ensure_file(self):
        with self._lock:
            self._ensure_file_locked()

    def _ensure_file_locked(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists() or self.path.stat().st_size == 0:
            self._write_header_locked()

    def _write_header_locked(self):
        with self.path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELDNAMES)
            writer.writeheader()