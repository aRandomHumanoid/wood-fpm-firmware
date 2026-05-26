"""High-level motion controller for the FPM.

Marlin handles the kinematics. The Pi just translates UI commands into
straight workspace-XY G-code and forwards them over serial.

Public surface (preserved across the EPOS→Marlin refactor so scan.py and
the web layer don't need to change):

    home_all()
    move_to(x, y, wait=True, ...)
    jog(dx, dy, ...)
    probe_to_x(target_x, y, ...)
    halt()              — controlled stop, drives stay enabled
    stop()              — E-stop, also disables steppers
    set_feed_mm_s(v)    — override feedrate for the next moves
    set_probe_state(b)  — push probe state into the broadcast loop
    is_busy() / state / subscribe(fn)
    start_polling(hz) / shutdown()

Units convention:
  - Workspace XY in mm — same as Marlin's internal units.
  - Feedrate F is mm/min (Marlin's standard), so v_mm_s × 60.
"""

from __future__ import annotations

import logging
import threading
import time
from datetime import datetime, timezone
from dataclasses import dataclass, asdict
from typing import Callable, Dict, Optional

from .limits import MachineBounds, MotionLimits, BoundsError
from ..drivers.marlin import MarlinError, make_marlin

log = logging.getLogger(__name__)


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    probe: bool = False
    homed: bool = False
    busy: bool = False
    fault: bool = False
    serial_connected: bool = False
    last_error: str = ""

    def to_dict(self) -> Dict:
        return asdict(self)


StateListener = Callable[[State], None]
SerialListener = Callable[[Dict[str, object]], None]


class MotionController:
    def __init__(
        self,
        bounds: MachineBounds,
        limits: MotionLimits,
        marlin_port: str = "/dev/ttyUSB0",
        marlin_baudrate: int = 115200,
        simulate: bool = False,
        auto_connect: bool = True,
        rapid_feed_mm_s: float = 15.0,
        scan_feed_mm_s: float = 2.0,
        x_axis: str = "X",
        y_axis: str = "Y",
        z_axis: str = "Z",
        position_tolerance_mm: float = 0.05,
    ):
        self.bounds = bounds
        self.limits = limits
        self.rapid_feed_mm_s = rapid_feed_mm_s
        self.scan_feed_mm_s = scan_feed_mm_s
        self._x_letter = x_axis
        self._y_letter = y_axis
        self._z_letter = z_axis
        self._pos_tol = position_tolerance_mm
        self._simulate = simulate
        self._marlin_port = marlin_port
        self._marlin_baudrate = marlin_baudrate
        self.marlin = None

        # Workspace feedrate currently in effect (for the next G1 we issue).
        self._feed_mm_s = limits.v_max_mm_s

        self.state = State()
        self._state_lock = threading.Lock()
        self._listeners: list[StateListener] = []
        self._serial_listeners: list[SerialListener] = []
        self._stop_evt = threading.Event()
        self._action_lock = threading.Lock()  # serializes home/move

        # is_busy tracking: each non-blocking move records its target; the
        # poll loop clears busy once the reported logical target is reached
        # and the stepper counts stop changing across consecutive polls.
        self._target: Optional[Dict[str, float]] = None
        self._busy_flag = False
        self._last_counts: Optional[Dict[str, int]] = None
        self._stable_count_polls = 0

        if auto_connect:
            try:
                self._connect_driver(
                    marlin_port=marlin_port,
                    marlin_baudrate=marlin_baudrate,
                    simulate=simulate,
                )
            except MarlinError as e:
                log.warning("initial Marlin connect failed: %s", e)
                self._set_state(fault=True, last_error=str(e), serial_connected=False)

    # -------- listeners / state --------

    def subscribe(self, fn: StateListener):
        self._listeners.append(fn)

    def subscribe_serial(self, fn: SerialListener):
        self._serial_listeners.append(fn)

    def _broadcast(self):
        with self._state_lock:
            snapshot = State(**self.state.to_dict())
        for fn in self._listeners:
            try:
                fn(snapshot)
            except Exception:
                log.exception("state listener raised")

    def _set_state(self, **kwargs):
        with self._state_lock:
            for k, v in kwargs.items():
                setattr(self.state, k, v)

    def _broadcast_serial(self, direction: str, line: str):
        entry = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "direction": direction,
            "line": line,
        }
        for fn in self._serial_listeners:
            try:
                fn(entry)
            except Exception:
                log.exception("serial listener raised")

    def serial_status(self) -> Dict[str, object]:
        with self._state_lock:
            connected = self.state.serial_connected
            last_error = self.state.last_error
        return {
            "connected": connected,
            "port": self._marlin_port,
            "baudrate": self._marlin_baudrate,
            "simulate": self._simulate,
            "last_error": last_error,
        }

    def connect_serial(
        self,
        port: Optional[str] = None,
        baudrate: Optional[int] = None,
        simulate: Optional[bool] = None,
    ) -> Dict[str, object]:
        with self._action_lock:
            target_port = port or self._marlin_port
            target_baudrate = int(baudrate or self._marlin_baudrate)
            target_simulate = self._simulate if simulate is None else bool(simulate)
            self._connect_driver(
                marlin_port=target_port,
                marlin_baudrate=target_baudrate,
                simulate=target_simulate,
            )
            self._target = None
            self._busy_flag = False
            self._last_counts = None
            self._stable_count_polls = 0
            self._set_state(fault=False, last_error="", serial_connected=True)
            self._broadcast()
            return self.serial_status()

    def disconnect_serial(self) -> Dict[str, object]:
        with self._action_lock:
            if self.marlin is not None:
                try:
                    self.marlin.close()
                except Exception:
                    pass
            self.marlin = None
            self._target = None
            self._busy_flag = False
            self._last_counts = None
            self._stable_count_polls = 0
            self._set_state(busy=False, homed=False, serial_connected=False)
            self._broadcast_serial("meta", f"disconnected from {self._marlin_port}")
            self._broadcast()
            return self.serial_status()

    def _connect_driver(self, marlin_port: str, marlin_baudrate: int, simulate: bool):
        new_marlin = make_marlin(
            simulate=simulate,
            port=marlin_port,
            baudrate=marlin_baudrate,
            transcript_hook=self._broadcast_serial,
        )
        try:
            new_marlin.set_absolute_mode()   # G90 — workspace coords from here on
            position = new_marlin.position()
        except Exception:
            try:
                new_marlin.close()
            except Exception:
                pass
            raise

        old_marlin = self.marlin
        self.marlin = new_marlin
        self._simulate = simulate
        self._marlin_port = marlin_port
        self._marlin_baudrate = marlin_baudrate
        self._set_state(
            x=position.logical.get(self._x_letter, self.state.x),
            y=position.logical.get(self._y_letter, self.state.y),
            z=position.logical.get(self._z_letter, self.state.z),
            busy=False,
            homed=False,
            serial_connected=True,
            fault=False,
        )
        if simulate:
            self._broadcast_serial("meta", "connected in simulation mode")
        else:
            self._broadcast_serial("meta", f"connected to {marlin_port} @ {marlin_baudrate}")
        if old_marlin is not None:
            try:
                old_marlin.close()
            except Exception:
                pass

    def _require_marlin(self):
        if self.marlin is None:
            raise MarlinError("serial not connected")
        return self.marlin

    def send_gcode(self, command: str, timeout_s: float = 30.0) -> list[str]:
        line = command.strip()
        if not line:
            raise ValueError("gcode command cannot be empty")
        with self._action_lock:
            return self._require_marlin().send(line, timeout_s=timeout_s)

    # -------- feedrate --------

    def set_feed_mm_s(self, v_mm_s: float):
        """Override workspace feedrate for the next G1."""
        self._feed_mm_s = max(min(v_mm_s, self.limits.v_max_mm_s), 0.01)

    def _feedrate_mm_min(self) -> float:
        return max(self._feed_mm_s * 60.0, 1.0)

    def _clear_pending_motion(self):
        self._target = None
        self._busy_flag = False
        self._last_counts = None
        self._stable_count_polls = 0

    # -------- public ops --------

    def home_all(self):
        with self._action_lock:
            marlin = self._require_marlin()
            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                axes = f"{self._x_letter} {self._y_letter}"
                log.info("homing axes: %s", axes)
                marlin.home(axes)
                self._target = None
                self._busy_flag = False
                self._refresh_position()
                self._set_state(homed=True, busy=False)
            except MarlinError as e:
                log.error("homing failed: %s", e)
                self._set_state(busy=False, fault=True, last_error=str(e))
                raise
            finally:
                self._broadcast()

    def move_to(self, x: float, y: float, wait: bool = True, timeout_s: float = 60.0):
        with self._action_lock:
            marlin = self._require_marlin()
            self.bounds.check(x, y)
            move_axes = {self._x_letter: x, self._y_letter: y}
            self._target = dict(move_axes)
            self._busy_flag = True
            self._last_counts = None
            self._stable_count_polls = 0
            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                marlin.move(move_axes, feedrate_mm_min=self._feedrate_mm_min())
                if wait:
                    self._wait_for_idle(timeout_s=timeout_s)
            except MarlinError as e:
                self._clear_pending_motion()
                self._set_state(busy=False, fault=True, last_error=str(e))
                raise
            finally:
                if wait:
                    self._set_state(busy=False)
                self._broadcast()

    def jog(self, dx: float, dy: float, **kwargs):
        """Relative workspace move."""
        pos = self._require_marlin().position()
        x = pos.logical.get(self._x_letter, self.state.x)
        y = pos.logical.get(self._y_letter, self.state.y)
        if self.state.homed:
            self.move_to(x + dx, y + dy, **kwargs)
            return

        wait = kwargs.pop("wait", True)
        timeout_s = kwargs.pop("timeout_s", 60.0)
        with self._action_lock:
            marlin = self._require_marlin()
            move_axes = {self._x_letter: dx, self._y_letter: dy}
            self._target = {self._x_letter: x + dx, self._y_letter: y + dy}
            self._busy_flag = True
            self._last_counts = None
            self._stable_count_polls = 0
            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                marlin.move_relative(move_axes, feedrate_mm_min=self._feedrate_mm_min())
                if wait:
                    self._wait_for_idle(timeout_s=timeout_s)
            except MarlinError as e:
                self._clear_pending_motion()
                self._set_state(busy=False, fault=True, last_error=str(e))
                raise
            finally:
                if wait:
                    self._set_state(busy=False)
                self._broadcast()

    def probe_to_x(self, target_x: float, y: float, timeout_s: float = 120.0) -> bool:
        """Probe toward ``target_x`` with a blocking G38.2 move.

        Returns ``True`` when Marlin stopped on probe contact and ``False``
        when the target was reached without triggering the probe.
        """
        with self._action_lock:
            marlin = self._require_marlin()
            self.bounds.check(target_x, y)
            self._target = None
            self._busy_flag = False
            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                probe_pos = marlin.probe_target(
                    {self._x_letter: target_x, self._y_letter: y},
                    feedrate_mm_min=self._feedrate_mm_min(),
                    timeout_s=timeout_s,
                )
                if probe_pos:
                    self._set_state(
                        x=probe_pos.get(self._x_letter, self.state.x),
                        y=probe_pos.get(self._y_letter, self.state.y),
                        z=probe_pos.get(self._z_letter, self.state.z),
                    )
                else:
                    self._refresh_position()
                return True
            except MarlinError as e:
                self._refresh_position()
                current_x = self.state.x
                current_y = self.state.y
                if (
                    abs(current_x - target_x) <= self._pos_tol
                    and abs(current_y - y) <= self._pos_tol
                ):
                    return False
                self._set_state(fault=True, last_error=str(e))
                raise
            finally:
                self._target = None
                self._busy_flag = False
                self._set_state(busy=False)
                self._broadcast()

    def halt(self):
        """Controlled mid-move stop. Drives remain enabled."""
        if self.marlin is None:
            return
        try:
            self.marlin.quick_stop()
            self._target = None
            self._busy_flag = False
            # Refresh state.y immediately so scan can grab the contact y.
            self._refresh_position()
        except MarlinError as e:
            log.warning("halt: %s", e)

    def stop(self):
        """E-stop: quick-stop + disable steppers."""
        if self.marlin is None:
            self._target = None
            self._busy_flag = False
            self._last_counts = None
            self._stable_count_polls = 0
            self._set_state(busy=False, fault=True, last_error="E-STOP")
            self._broadcast()
            return
        try:
            self.marlin.quick_stop()
        except MarlinError as e:
            log.warning("stop quick_stop: %s", e)
        try:
            self.marlin.disable_steppers()
        except MarlinError as e:
            log.warning("stop disable: %s", e)
        self._target = None
        self._busy_flag = False
        self._last_counts = None
        self._stable_count_polls = 0
        self._set_state(busy=False, fault=True, last_error="E-STOP")
        self._broadcast_serial("meta", "E-STOP latched")
        self._broadcast()

    def _clear_fault_latch(self, transcript_line: str):
        self._target = None
        self._busy_flag = False
        self._last_counts = None
        self._stable_count_polls = 0
        self._set_state(busy=False, fault=False, last_error="")
        self._broadcast_serial("meta", transcript_line)

    def clear_fault(self) -> bool:
        """Clear the local fault or busy latch without reconnecting serial."""
        with self._action_lock:
            with self._state_lock:
                clearable = (
                    self.state.busy
                    or self.state.fault
                    or bool(self.state.last_error)
                    or self._busy_flag
                    or self._target is not None
                )
            if not clearable:
                return False
            self._clear_fault_latch("Fault cleared")
        self._broadcast()
        return True

    def reset_estop(self) -> bool:
        """Clear the local E-STOP latch without reconnecting serial."""
        with self._action_lock:
            with self._state_lock:
                estop_latched = self.state.last_error == "E-STOP"
            if not estop_latched:
                return False
            self._clear_fault_latch("E-STOP reset")
        self._broadcast()
        return True

    def is_busy(self) -> bool:
        with self._state_lock:
            return self.state.busy

    # -------- busy / idle tracking --------

    def _wait_for_idle(self, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._refresh_position():
                if not self._busy_flag:
                    return
            else:
                time.sleep(0.05)
                continue
            time.sleep(0.03)
        raise MarlinError("timeout waiting for move to complete")

    # -------- polling --------

    def start_polling(self, hz: float = 5.0):
        self._poll_thread = threading.Thread(target=self._poll_loop, args=(hz,), daemon=True)
        self._poll_thread.start()

    def shutdown(self):
        self._stop_evt.set()
        if self.marlin is not None:
            try:
                self.marlin.disable_steppers()
            except Exception:
                pass
            try:
                self.marlin.close()
            except Exception:
                pass
        self.marlin = None

    def _poll_loop(self, hz: float):
        period = 1.0 / hz
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                self._refresh_position()
                self._broadcast()
            except Exception:
                log.exception("poll loop")
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))

    def _refresh_position(self) -> bool:
        """Query Marlin and update self.state. Returns True on success."""
        if self.marlin is None:
            return False
        try:
            pos = self.marlin.position()
        except MarlinError as e:
            log.debug("M114 failed: %s", e)
            return False

        x = pos.logical.get(self._x_letter, self.state.x)
        y = pos.logical.get(self._y_letter, self.state.y)
        z = pos.logical.get(self._z_letter, self.state.z)

        # Busy detection: stay busy until the reported position is within
        # tolerance of the commanded target on every commanded axis.
        if self._target is not None:
            xy_pos = {self._x_letter: x, self._y_letter: y}
            logical_done = all(
                abs(xy_pos.get(ax, t) - t) <= self._pos_tol
                for ax, t in self._target.items()
            )
            tracked_counts = {
                ax: pos.counts[ax]
                for ax in self._target
                if ax in pos.counts
            }
            if logical_done and tracked_counts:
                if self._last_counts == tracked_counts:
                    self._stable_count_polls += 1
                else:
                    self._stable_count_polls = 0
                self._last_counts = tracked_counts
            elif logical_done:
                self._last_counts = None
                self._stable_count_polls = 1
            else:
                self._last_counts = tracked_counts or None
                self._stable_count_polls = 0

            if logical_done and self._stable_count_polls >= 1:
                self._busy_flag = False
                self._target = None
                self._last_counts = None
                self._stable_count_polls = 0

        self._set_state(x=x, y=y, z=z, busy=self._busy_flag)
        return True

    # -------- probe binding --------

    def set_probe_state(self, triggered: bool):
        self._set_state(probe=triggered)
        self._broadcast()
