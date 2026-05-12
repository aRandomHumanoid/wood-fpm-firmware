"""High-level motion controller for the FPM.

Orchestrates the kinematics + EPOS2 driver. Exposes the operations the web UI
needs: home_all, move_to, jog, stop, current state. Owns a background thread
that polls position and broadcasts state to subscribers.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, asdict
from typing import Callable, Dict, Optional

from .kinematics import PPMKinematics, KinematicsResult
from .limits import MachineBounds, MotionLimits, BoundsError, OvercurrentError
from ..drivers.epos import (
    EposError,
    HOMING_INDEX_POS,
    HOMING_INDEX_NEG,
    make_axis,
    make_driver,
)

log = logging.getLogger(__name__)


@dataclass
class AxisConfig:
    name: str
    epos_node: int
    port_name: str        # Maxon USB port label, e.g. "USB0" / "USB1"
    counts_per_rev: int   # EPOS-reported counts per motor rev (encoder cpt × 4 for quadrature)
    gear_ratio: float     # planetary reduction, motor -> joint
    invert: bool
    home_method: int
    home_speed_rpm: int
    home_accel_rpm_s: int
    home_current_threshold_mA: int = 0  # only used by current-threshold methods (-1..-4)

    @property
    def counts_per_deg(self) -> float:
        return self.gear_ratio * self.counts_per_rev / 360.0

    def deg_to_counts(self, deg: float) -> int:
        v = deg * self.counts_per_deg
        return int(round(-v if self.invert else v))

    def counts_to_deg(self, counts: int) -> float:
        v = counts / self.counts_per_deg
        return -v if self.invert else v


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    theta_deg: float = 0.0
    phi_deg: float = 0.0
    probe: bool = False
    homed: bool = False
    busy: bool = False
    fault: bool = False
    last_error: str = ""

    def to_dict(self) -> Dict:
        return asdict(self)


StateListener = Callable[[State], None]


class MotionController:
    def __init__(
        self,
        kin: PPMKinematics,
        bounds: MachineBounds,
        limits: MotionLimits,
        axis_theta: AxisConfig,
        axis_phi: AxisConfig,
        simulate: bool = False,
        epos_library: str = "libEposCmd.so.6.6.1.0",
        rapid_feed_mm_s: float = 15.0,
        scan_feed_mm_s: float = 2.0,
    ):
        self.kin = kin
        self.bounds = bounds
        self.limits = limits
        self.axis_theta_cfg = axis_theta
        self.axis_phi_cfg = axis_phi
        self.rapid_feed_mm_s = rapid_feed_mm_s
        self.scan_feed_mm_s = scan_feed_mm_s

        # Each EPOS2 gateway has its own USB cable, so each gets its own driver.
        self.driver_theta = make_driver(
            simulate=simulate,
            library_path=epos_library,
            port_name=axis_theta.port_name,
        )
        self.driver_phi = make_driver(
            simulate=simulate,
            library_path=epos_library,
            port_name=axis_phi.port_name,
        )
        self.axis_theta = make_axis(self.driver_theta, axis_theta.epos_node)
        self.axis_phi = make_axis(self.driver_phi, axis_phi.epos_node)

        self.state = State()
        self._state_lock = threading.Lock()
        self._listeners: list[StateListener] = []
        self._stop_evt = threading.Event()
        self._action_lock = threading.Lock()  # serializes home/move/scan

        self._configure_axes()

    # -------- setup --------

    def _configure_axes(self):
        for axis, cfg in ((self.axis_theta, self.axis_theta_cfg), (self.axis_phi, self.axis_phi_cfg)):
            try:
                if hasattr(axis, "clear_fault"):
                    axis.clear_fault()
                axis.enable()
                axis.activate_pp_mode()
                v_rpm, a_rpm_s = self._motor_profile_from_workspace()
                axis.set_position_profile(v_rpm, a_rpm_s, a_rpm_s)
            except EposError as e:
                log.error("axis %s setup: %s", cfg.name, e)
                self._set_state(fault=True, last_error=str(e))

    def _motor_profile_from_workspace(self) -> tuple[int, int]:
        """Map workspace vel/accel limits to motor RPM / RPM/s via a small-signal
        Jacobian at origin (dx/dtheta ~ Lc). Conservative; can be tuned later."""
        Lc = self.kin.L_char
        if Lc <= 0:
            return 100, 1000
        # joint rad/s ~ v_mm_s / Lc  ->  joint deg/s = (v / Lc) * 180/pi
        joint_deg_s = (self.limits.v_max_mm_s / Lc) * (180.0 / math.pi)
        # output shaft RPM = deg/s * 60 / 360
        output_rpm = joint_deg_s * 60.0 / 360.0
        # motor RPM (pre-gearbox)
        motor_rpm = output_rpm * self.axis_theta_cfg.gear_ratio
        # similar for accel
        joint_deg_s2 = (self.limits.a_max_mm_s2 / Lc) * (180.0 / math.pi)
        motor_rpm_s = joint_deg_s2 * 60.0 / 360.0 * self.axis_theta_cfg.gear_ratio
        return max(int(motor_rpm), 1), max(int(motor_rpm_s), 1)

    # -------- listeners / state --------

    def subscribe(self, fn: StateListener):
        self._listeners.append(fn)

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

    # -------- public ops --------

    def home_all(self):
        with self._action_lock:
            self._set_state(busy=True, last_error="")
            try:
                for axis, cfg in (
                    (self.axis_theta, self.axis_theta_cfg),
                    (self.axis_phi, self.axis_phi_cfg),
                ):
                    if hasattr(axis, "clear_fault"):
                        axis.clear_fault()
                    axis.enable()
                    axis.activate_homing_mode()
                    axis.set_homing_parameter(
                        method=cfg.home_method,
                        home_speed_rpm=cfg.home_speed_rpm,
                        acceleration_rpm_s=cfg.home_accel_rpm_s,
                        current_threshold_mA=cfg.home_current_threshold_mA,
                    )
                    axis.find_home(timeout_s=120.0)
                    axis.activate_pp_mode()
                    v_rpm, a_rpm_s = self._motor_profile_from_workspace()
                    axis.set_position_profile(v_rpm, a_rpm_s, a_rpm_s)
                self._set_state(homed=True, busy=False)
            except EposError as e:
                log.error("homing failed: %s", e)
                self._set_state(busy=False, fault=True, last_error=str(e))
                raise
            finally:
                self._broadcast()

    def move_to(self, x: float, y: float, wait: bool = True, timeout_s: float = 60.0):
        with self._action_lock:
            self.bounds.check(x, y)
            ik: KinematicsResult = self.kin.inverse_kinematics(x, y)
            if not ik.ok:
                raise BoundsError(f"kinematics unreachable: {ik.reason}")

            theta_counts = self.axis_theta_cfg.deg_to_counts(ik.theta_deg)
            phi_counts = self.axis_phi_cfg.deg_to_counts(ik.phi_deg)

            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                self.axis_theta.move_to(theta_counts, absolute=True)
                self.axis_phi.move_to(phi_counts, absolute=True)
                if wait:
                    self._wait_done_with_current_guard(timeout_s=timeout_s)
            except (EposError, OvercurrentError) as e:
                self._set_state(fault=True, last_error=str(e))
                raise
            finally:
                self._set_state(busy=False)
                self._broadcast()

    def _wait_done_with_current_guard(self, timeout_s: float, poll_s: float = 0.02):
        """Block until both axes report target reached, halting and raising
        OvercurrentError if either axis's |current| exceeds the configured
        limit. Falls through to plain waits if no limit is configured."""
        limit = self.limits.overcurrent_mA
        if limit <= 0:
            self.axis_theta.wait_done(timeout_s=timeout_s)
            self.axis_phi.wait_done(timeout_s=timeout_s)
            return

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for axis, name in ((self.axis_theta, "theta"), (self.axis_phi, "phi")):
                try:
                    cur = axis.current_mA()
                except EposError:
                    continue  # transient read failure; retry next tick
                if abs(cur) > limit:
                    self.halt()
                    raise OvercurrentError(
                        f"{name} drew {cur} mA (>{limit} mA); move aborted"
                    )
            if self.axis_theta.target_reached() and self.axis_phi.target_reached():
                return
            time.sleep(poll_s)
        raise EposError("_wait_done_with_current_guard", 0, "timeout waiting for axes")

    def jog(self, dx: float, dy: float, **kwargs):
        with self._state_lock:
            x, y = self.state.x + dx, self.state.y + dy
        self.move_to(x, y, **kwargs)

    def halt(self):
        """Halt motion in place WITHOUT disabling drives. Used mid-scan."""
        for axis in (self.axis_theta, self.axis_phi):
            try:
                axis.halt()
            except EposError as e:
                log.warning("halt: %s", e)

    def stop(self):
        """E-stop: halt and disable drives."""
        self.halt()
        try:
            self.axis_theta.disable()
            self.axis_phi.disable()
        except EposError as e:
            log.warning("disable on stop: %s", e)
        self._set_state(busy=False, last_error="E-STOP")
        self._broadcast()

    def set_feed_mm_s(self, v_mm_s: float):
        """Override the per-axis profile velocity. Pass v_max_mm_s to restore."""
        Lc = self.kin.L_char
        if Lc <= 0:
            return
        joint_deg_s = (v_mm_s / Lc) * (180.0 / math.pi)
        output_rpm = joint_deg_s * 60.0 / 360.0
        for axis, cfg in (
            (self.axis_theta, self.axis_theta_cfg),
            (self.axis_phi, self.axis_phi_cfg),
        ):
            motor_rpm = max(int(output_rpm * cfg.gear_ratio), 1)
            joint_deg_s2 = (self.limits.a_max_mm_s2 / Lc) * (180.0 / math.pi)
            motor_rpm_s = max(int(joint_deg_s2 * 60.0 / 360.0 * cfg.gear_ratio), 1)
            try:
                axis.set_position_profile(motor_rpm, motor_rpm_s, motor_rpm_s)
            except EposError as e:
                log.warning("set_position_profile: %s", e)

    def is_busy(self) -> bool:
        with self._state_lock:
            return self.state.busy

    # -------- polling thread --------

    def start_polling(self, hz: float = 20.0):
        self._poll_thread = threading.Thread(target=self._poll_loop, args=(hz,), daemon=True)
        self._poll_thread.start()

    def shutdown(self):
        self._stop_evt.set()
        try:
            self.axis_theta.disable()
            self.axis_phi.disable()
        except Exception:
            pass
        for drv in (self.driver_theta, self.driver_phi):
            if hasattr(drv, "close"):
                try:
                    drv.close()
                except Exception:
                    pass

    def _poll_loop(self, hz: float):
        period = 1.0 / hz
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                theta_counts = self.axis_theta.position()
                phi_counts = self.axis_phi.position()
                theta_deg = self.axis_theta_cfg.counts_to_deg(theta_counts)
                phi_deg = self.axis_phi_cfg.counts_to_deg(phi_counts)
                x, y, z = self.kin.forward_kinematics(
                    math.radians(theta_deg), math.radians(phi_deg)
                )
                # busy = any axis still moving
                busy = not (self.axis_theta.target_reached() and self.axis_phi.target_reached())
                fault = False
                try:
                    if hasattr(self.axis_theta, "fault_state"):
                        fault = self.axis_theta.fault_state() or self.axis_phi.fault_state()
                except Exception:
                    pass
                self._set_state(
                    x=x, y=y, z=z,
                    theta_deg=theta_deg, phi_deg=phi_deg,
                    busy=busy, fault=fault,
                )
                self._broadcast()
            except Exception:
                log.exception("poll loop")
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))

    # -------- probe binding --------

    def set_probe_state(self, triggered: bool):
        self._set_state(probe=triggered)
        self._broadcast()
