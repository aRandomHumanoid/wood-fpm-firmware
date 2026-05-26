"""Marlin serial G-code driver.

The Pi talks to a Marlin mainboard over a serial link (typically USB CDC at
115200 or 250000 baud). Marlin processes G-code one command at a time and
acknowledges each with an ``ok`` line.

Marlin caveats this driver deals with:

* Position queries (M114) need to come back promptly even while moves are
  queued. This works on stock Marlin because M114 doesn't generate motion,
  but builds without ``EMERGENCY_PARSER`` may delay the ``ok`` slightly.
* M114 returns two relevant fields:
    - ``X:N.N Y:N.N ...`` — the planner's logical position (set as soon as a
      G1 is parsed, *before* steppers finish moving).
    - ``Count X:N Y:N ...`` — the actual stepper-step position.
  We use the logical position for state display and the stepper count to
  detect when a move has fully completed.
* All serial I/O is funneled through a single worker thread fed by a queue,
  because a single serial channel can't be safely shared by multiple
  callers expecting their own ``ok``.

A ``SimulatedMarlinDriver`` mirrors the same interface for off-hardware
testing.
"""

from __future__ import annotations

import logging
import queue
import re
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

log = logging.getLogger(__name__)


class MarlinError(RuntimeError):
    pass


@dataclass
class Position:
    """Snapshot of axis positions from M114."""
    logical: Dict[str, float]   # 'X' -> mm-equivalent (joint deg)
    counts:  Dict[str, int]     # 'X' -> raw stepper steps

    def settled(self, tol_counts: int = 2) -> bool:
        """Returns True iff every axis whose logical position is non-finite-
        check'd has its stepper count consistent with the logical target.

        Marlin's M114 shows logical position = planner end-of-queue and
        counts = current stepper position. When the move is done they
        agree (modulo steps_per_mm rounding). We can't recompute the
        steps_per_mm here without round-tripping with the board, so this
        helper is only a stub — callers do their own comparison.
        """
        return True


# Regexes for parsing M114 output. Examples:
#   "X:0.00 Y:0.00 Z:0.00 E:0.00 Count X:0 Y:0 Z:0"
_LOGICAL_RE = re.compile(r"\b([XYZABCUVWE])\s*:\s*(-?\d+\.?\d*)")
_COUNT_RE   = re.compile(r"Count\s+([XYZABCUVWE])\s*:\s*(-?\d+)")
_GCODE_AXIS_RE = re.compile(r"\b([XYZABCUVWE])\s*(-?\d+\.?\d*)")


def parse_m114(lines: List[str]) -> Position:
    """Parse the response lines from an M114 command."""
    logical: Dict[str, float] = {}
    counts:  Dict[str, int]   = {}
    for line in lines:
        # Logical positions: pull only the first chunk before "Count".
        head = line.split("Count", 1)[0]
        for axis, val in _LOGICAL_RE.findall(head):
            # Skip E* (extruder) — the FPM doesn't use it but Marlin reports it.
            if axis == "E":
                continue
            logical.setdefault(axis, float(val))
        for axis, val in _COUNT_RE.findall(line):
            counts.setdefault(axis, int(val))
    return Position(logical=logical, counts=counts)


class MarlinDriver:
    """One serial connection to one Marlin board.

    ``send(cmd)`` is synchronous: it enqueues the command, the I/O worker
    writes it to the wire and reads lines until an ``ok`` (or error)
    appears, and returns the lines to the caller. Multiple threads can call
    ``send`` concurrently — they serialize through the queue.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        connect_timeout_s: float = 8.0,
        read_timeout_s: float = 0.2,
        transcript_hook: Optional[Callable[[str, str], None]] = None,
    ):
        import serial  # imported lazily so the package can install without pyserial on dev hosts
        self._serial = serial.Serial(port, baudrate, timeout=read_timeout_s)
        self._cmd_q: "queue.Queue[tuple[str, queue.Queue[list[str] | MarlinError]]]" = queue.Queue()
        self._running = True
        self._transcript_hook = transcript_hook
        self._io_thread = threading.Thread(target=self._io_loop, daemon=True, name="marlin-io")
        self._io_thread.start()
        self._wait_for_ready(connect_timeout_s)
        log.info("Marlin connected on %s @ %d baud", port, baudrate)

    def _emit_transcript(self, direction: str, line: str):
        if self._transcript_hook is None:
            return
        try:
            self._transcript_hook(direction, line)
        except Exception:
            log.exception("serial transcript hook raised")

    # ----- low-level I/O -----

    def _wait_for_ready(self, timeout_s: float):
        """Marlin emits ``start`` on power-up or USB-reset; some builds also
        send a banner. Either way an M115 probe nails the ready state."""
        deadline = time.monotonic() + timeout_s
        # Drain any unsolicited startup chatter.
        while time.monotonic() < deadline:
            line = self._serial.readline().decode(errors="replace").strip()
            if not line:
                if self._serial.in_waiting == 0:
                    break
                continue
            self._emit_transcript("rx", line)
            log.debug("marlin boot: %s", line)
            if line == "start" or line.lower().startswith("echo:start"):
                break
        try:
            self.send("M115", timeout_s=timeout_s)
        except MarlinError as e:
            raise MarlinError(f"failed to reach Marlin on serial: {e}") from e

    def _io_loop(self):
        while self._running:
            try:
                cmd, reply_q = self._cmd_q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._emit_transcript("tx", cmd.strip())
                self._serial.write((cmd.strip() + "\n").encode())
                self._serial.flush()
                lines = self._read_until_ok()
                reply_q.put(lines)
            except Exception as e:
                reply_q.put(MarlinError(str(e)))

    def _read_until_ok(self, overall_timeout_s: float = 30.0) -> List[str]:
        deadline = time.monotonic() + overall_timeout_s
        lines: List[str] = []
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue  # readline timed out, keep waiting
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            self._emit_transcript("rx", line)
            lines.append(line)
            low = line.lower()
            if low == "ok" or low.startswith("ok "):
                return lines
            if low.startswith("error") or low.startswith("!!"):
                raise MarlinError(f"Marlin: {line}")
        raise MarlinError(f"timeout (last lines: {lines[-3:]})")

    # ----- public API -----

    def send(self, cmd: str, timeout_s: float = 30.0) -> List[str]:
        """Send a single G-code line, block until ``ok`` (or raise)."""
        reply_q: "queue.Queue[list[str] | MarlinError]" = queue.Queue()
        self._cmd_q.put((cmd, reply_q))
        try:
            result = reply_q.get(timeout=timeout_s)
        except queue.Empty:
            raise MarlinError(f"send timeout: {cmd}")
        if isinstance(result, MarlinError):
            raise result
        return result

    def position(self) -> Position:
        return parse_m114(self.send("M114"))

    def move(self, axes: Dict[str, float], feedrate_mm_min: Optional[float] = None):
        """Issue a G1 absolute move. ``axes`` like ``{'X': 12.5, 'Y': -3.0}``.
        Returns immediately once Marlin queues the move (the planner is
        still draining when send returns)."""
        parts = [f"{k}{v:.4f}" for k, v in axes.items()]
        if feedrate_mm_min is not None:
            parts.append(f"F{feedrate_mm_min:.1f}")
        self.send("G1 " + " ".join(parts))

    def probe_target(
        self,
        axes: Dict[str, float],
        feedrate_mm_min: Optional[float] = None,
        timeout_s: float = 300.0,
    ):
        """Issue a blocking G38.2 probe move toward the supplied target."""
        parts = [f"{k}{v:.4f}" for k, v in axes.items()]
        if feedrate_mm_min is not None:
            parts.append(f"F{feedrate_mm_min:.1f}")
        self.send("G38.2 " + " ".join(parts), timeout_s=timeout_s)

    def home(self, axes: str = "X Y"):
        """Block until G28 reports ``ok`` (Marlin doesn't ack until done)."""
        self.send(f"G28 {axes}", timeout_s=300.0)

    def quick_stop(self):
        """M410 — controlled halt, drops the planner buffer."""
        try:
            self.send("M410", timeout_s=5.0)
        except MarlinError:
            # Some builds don't ack M410 cleanly when motion is active.
            log.warning("M410 did not ack normally (continuing)")

    def disable_steppers(self):
        self.send("M84")

    def set_absolute_mode(self):
        self.send("G90")

    def set_max_feedrate(self, axes: Dict[str, float]):
        parts = [f"{k}{v:.2f}" for k, v in axes.items()]
        self.send("M203 " + " ".join(parts))

    def set_max_accel(self, axes: Dict[str, float]):
        parts = [f"{k}{v:.1f}" for k, v in axes.items()]
        self.send("M201 " + " ".join(parts))

    def close(self):
        self._running = False
        try:
            self._serial.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------


class SimulatedMarlinDriver:
    """Software-only stand-in. Tracks per-axis positions that snap to the
    commanded targets after a brief simulated travel time. Honors quick_stop
    to freeze motion mid-flight."""

    AXES = ("X", "Y", "Z", "A", "B", "C")

    def __init__(
        self,
        probe_surface_x: Optional[Callable[[float], Optional[float]]] = None,
        transcript_hook: Optional[Callable[[str, str], None]] = None,
        **_kwargs,
    ):
        self._lock = threading.Lock()
        self._pos:    Dict[str, float] = {a: 0.0 for a in self.AXES}
        self._target: Dict[str, float] = {a: 0.0 for a in self.AXES}
        self._move_end: float = 0.0
        self._move_start: float = 0.0
        self._start_pos: Dict[str, float] = {a: 0.0 for a in self.AXES}
        self._feedrate_mm_min: float = 600.0   # 10 mm/s default
        self._homed = False
        self._probe_surface_x = probe_surface_x
        self._transcript_hook = transcript_hook

    def _emit_transcript(self, direction: str, line: str):
        if self._transcript_hook is None:
            return
        self._transcript_hook(direction, line)

    # ----- shared API -----

    def send(self, cmd: str, timeout_s: float = 30.0) -> List[str]:
        self._emit_transcript("tx", cmd.strip())
        # Crude: dispatch on the first token.
        tok = cmd.strip().split()[0].upper()
        if tok == "M115":
            lines = ["FIRMWARE_NAME:Marlin-sim", "ok"]
            for line in lines:
                self._emit_transcript("rx", line)
            return lines
        if tok == "M114":
            self._tick()
            lines = [self._format_m114(), "ok"]
            for line in lines:
                self._emit_transcript("rx", line)
            return lines
        if tok in ("M84", "M410", "M201", "M203", "M205", "M220", "G90", "G91"):
            if tok == "M410":
                self._tick()
                self._target = dict(self._pos)
                self._move_end = self._move_start = 0.0
            self._emit_transcript("rx", "ok")
            return ["ok"]
        if tok == "G28":
            with self._lock:
                self._pos = {a: 0.0 for a in self.AXES}
                self._target = dict(self._pos)
                self._start_pos = dict(self._pos)
                self._move_end = self._move_start = 0.0
                self._homed = True
            self._emit_transcript("rx", "ok")
            return ["ok"]
        if tok == "G1" or tok == "G0":
            self._handle_g1(cmd)
            self._emit_transcript("rx", "ok")
            return ["ok"]
        if tok == "G38.2":
            axes = {axis: float(val) for axis, val in _GCODE_AXIS_RE.findall(cmd.upper())}
            m_f = re.search(r"\bF\s*(\d+\.?\d*)", cmd.upper())
            feedrate = float(m_f.group(1)) if m_f else None
            self.probe_target(axes, feedrate_mm_min=feedrate, timeout_s=timeout_s)
            self._emit_transcript("rx", "ok")
            return ["ok"]
        self._emit_transcript("rx", "ok")
        return ["ok"]

    def position(self) -> Position:
        self._tick()
        with self._lock:
            return Position(
                logical=dict(self._target),
                counts={a: int(round(self._pos[a] * 100)) for a in self.AXES},
            )

    def move(self, axes: Dict[str, float], feedrate_mm_min: Optional[float] = None):
        parts = [f"{k}{v}" for k, v in axes.items()]
        if feedrate_mm_min is not None:
            parts.append(f"F{feedrate_mm_min}")
        self.send("G1 " + " ".join(parts))

    def probe_target(
        self,
        axes: Dict[str, float],
        feedrate_mm_min: Optional[float] = None,
        timeout_s: float = 300.0,
    ):
        del timeout_s
        self._tick()
        if feedrate_mm_min is not None:
            self._feedrate_mm_min = float(feedrate_mm_min)

        with self._lock:
            start_x = self._pos["X"]
            target_x = float(axes.get("X", start_x))
            target_y = float(axes.get("Y", self._pos["Y"]))

            contact_x: Optional[float] = None
            if callable(self._probe_surface_x) and "X" in axes:
                candidate = self._probe_surface_x(target_y)
                lo, hi = sorted((start_x, target_x))
                if candidate is not None and lo <= candidate <= hi:
                    contact_x = float(candidate)

            final_x = contact_x if contact_x is not None else target_x
            self._target["X"] = final_x
            self._target["Y"] = target_y
            self._pos["X"] = final_x
            self._pos["Y"] = target_y
            self._move_end = self._move_start = 0.0

        if contact_x is None:
            raise MarlinError("G38.2 target reached without probe trigger")

    def home(self, axes: str = "X Y"):
        self.send("G28 " + axes)

    def quick_stop(self):
        self.send("M410")

    def disable_steppers(self):
        self.send("M84")

    def set_absolute_mode(self):
        pass

    def set_max_feedrate(self, axes: Dict[str, float]):
        pass

    def set_max_accel(self, axes: Dict[str, float]):
        pass

    def close(self):
        pass

    # ----- internals -----

    def _handle_g1(self, cmd: str):
        targets: Dict[str, float] = {}
        for axis, val in _GCODE_AXIS_RE.findall(cmd.upper()):
            if axis in self.AXES:
                targets[axis] = float(val)
        m_f = re.search(r"\bF\s*(\d+\.?\d*)", cmd.upper())
        if m_f:
            self._feedrate_mm_min = float(m_f.group(1))

        self._tick()
        with self._lock:
            self._start_pos = dict(self._pos)
            for a, v in targets.items():
                self._target[a] = v
            # Distance and duration estimate.
            d2 = sum((self._target[a] - self._start_pos[a]) ** 2 for a in targets)
            distance = d2 ** 0.5
            mm_per_s = self._feedrate_mm_min / 60.0
            duration = distance / mm_per_s if mm_per_s > 0 and distance > 0 else 0.0
            now = time.monotonic()
            self._move_start = now
            self._move_end = now + duration

    def _tick(self):
        with self._lock:
            if self._move_end <= self._move_start:
                self._pos = dict(self._target)
                return
            now = time.monotonic()
            if now >= self._move_end:
                self._pos = dict(self._target)
                self._move_end = self._move_start = 0.0
                return
            f = (now - self._move_start) / (self._move_end - self._move_start)
            for a in self.AXES:
                self._pos[a] = self._start_pos[a] + f * (self._target[a] - self._start_pos[a])

    def _format_m114(self) -> str:
        logical = " ".join(f"{a}:{self._target[a]:.3f}" for a in ("X", "Y", "Z"))
        counts  = "Count " + " ".join(f"{a}:{int(round(self._pos[a] * 100))}" for a in ("X", "Y", "Z"))
        return f"{logical} E:0.000 {counts}"


def make_marlin(simulate: bool, **kwargs):
    if simulate:
        return SimulatedMarlinDriver(**kwargs)
    return MarlinDriver(**kwargs)
