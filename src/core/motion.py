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


# EPOS homing methods grouped by the direction the motor spins to find home.
_POSITIVE_HOMING_METHODS = frozenset({34, 23, 7, 2, 18, -1, -3})
_NEGATIVE_HOMING_METHODS = frozenset({33, 27, 11, 1, 17, -2, -4})


def _homing_direction(method: int) -> int:
    """Return +1 if `method` homes in the positive motor direction, -1 if
    negative, 0 if neither (e.g. HM_ACTUAL_POSITION = 35, which doesn't move)."""
    if method in _POSITIVE_HOMING_METHODS:
        return +1
    if method in _NEGATIVE_HOMING_METHODS:
        return -1
    return 0


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
    motor_velocity_rpm: int = 100       # default PP-mode profile velocity (motor-shaft)
    motor_accel_rpm_s: int = 1000       # default PP-mode profile accel/decel
    joint_travel_deg: float = 90.0      # allowed travel from home position
    # Encoder joint angle (in degrees, post-gear) at which the platform sits
    # in the KINEMATICS neutral pose. 0.0 means home itself is the neutral
    # pose; for a joint homed to one corner of a symmetric travel, set this
    # to joint_travel_deg / 2 (e.g. 30° for a 60° travel range).
    home_offset_deg: float = 0.0
    # After the EPOS reports homing-attained, move this many joint degrees
    # AWAY from the hard stop and then re-zero the encoder there. The hard
    # stop ends up just outside the post-homing position limits, so encoder
    # 0 (and any "rotate abs theta 0" command) is safe to hold/return to
    # instead of slamming back into the mechanism.
    home_back_off_deg: float = 0.0
    # Margin past the home/back-off point (toward the hard-stop side) that
    # the position limit allows. The controller commonly settles a handful
    # of counts past 0 while returning to home; without a margin those
    # counts trip the EPOS limit and raise BoundsError on benign moves
    # like the second half of `wiggle`.
    home_overshoot_margin_deg: float = 0.0

    @property
    def counts_per_deg(self) -> float:
        return self.gear_ratio * self.counts_per_rev / 360.0

    # ENCODER frame: joint degrees as the motor encoder reports them
    # (0 = home position). Used by rotate_axis, status, EPOS software
    # position limits.
    def deg_to_counts(self, deg: float) -> int:
        v = deg * self.counts_per_deg
        return int(round(-v if self.invert else v))

    def counts_to_deg(self, counts: int) -> float:
        v = counts / self.counts_per_deg
        return -v if self.invert else v

    # KINEMATICS frame: joint degrees in the IK frame (0 = neutral pose).
    # Offset by ``home_offset_deg`` from the encoder frame. Used by
    # move_to / forward kinematics.
    def kinematics_deg_to_counts(self, kdeg: float) -> int:
        return self.deg_to_counts(kdeg + self.home_offset_deg)

    def counts_to_kinematics_deg(self, counts: int) -> float:
        return self.counts_to_deg(counts) - self.home_offset_deg


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
        # Per-axis position limits in motor counts; populated after home_all.
        # None means "not yet homed — no limit applied".
        self._pos_limits: dict[str, tuple[int, int] | None] = {"theta": None, "phi": None}

        self._configure_axes()

    # -------- setup --------

    def _configure_axes(self):
        for axis, cfg in ((self.axis_theta, self.axis_theta_cfg), (self.axis_phi, self.axis_phi_cfg)):
            try:
                if hasattr(axis, "clear_fault"):
                    axis.clear_fault()
                axis.enable()
                axis.activate_pp_mode()
                axis.set_position_profile(
                    cfg.motor_velocity_rpm,
                    cfg.motor_accel_rpm_s,
                    cfg.motor_accel_rpm_s,
                )
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
                    # If a back-off is configured, encode it into the EPOS's
                    # HomeOffset so the drive itself: (1) drives to stop,
                    # (2) backs off by HomeOffset counts, (3) zeroes at the
                    # back-off point — all in homing mode, one atomic routine.
                    direction = _homing_direction(cfg.home_method)
                    back_off_counts = 0
                    if direction != 0 and cfg.home_back_off_deg > 0:
                        back_off_counts = -direction * int(round(
                            cfg.home_back_off_deg * cfg.counts_per_deg
                        ))
                    axis.set_homing_parameter(
                        method=cfg.home_method,
                        home_speed_rpm=cfg.home_speed_rpm,
                        acceleration_rpm_s=cfg.home_accel_rpm_s,
                        offset_counts=back_off_counts,
                        current_threshold_mA=cfg.home_current_threshold_mA,
                    )
                    log.info("axis %s: homing (method=%d, back_off=%d counts)",
                             cfg.name, cfg.home_method, back_off_counts)
                    axis.find_home(timeout_s=120.0)
                    # Hardstop homing usually trips a Following Error when
                    # the joint stalls against the mechanism just before
                    # the current threshold fires, which leaves the drive
                    # in a Fault state (disabled). Clear it and re-enable
                    # so the axis is ready for subsequent moves.
                    if hasattr(axis, "clear_fault"):
                        axis.clear_fault()
                    axis.enable()
                    axis.activate_pp_mode()
                    axis.set_position_profile(
                        cfg.motor_velocity_rpm,
                        cfg.motor_accel_rpm_s,
                        cfg.motor_accel_rpm_s,
                    )
                    # Issue a hold target at the current position so the
                    # PP-mode controller is actively maintaining it.
                    pos_after_home = axis.position()
                    axis.move_to(pos_after_home, absolute=True)
                    log.info("axis %s: post-homing position = %d counts",
                             cfg.name, pos_after_home)
                    self._apply_position_limits(axis, cfg)
                    if hasattr(axis, "fault_state") and axis.fault_state():
                        log.warning("axis %s still faulted after homing", cfg.name)
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

            theta_counts = self.axis_theta_cfg.kinematics_deg_to_counts(ik.theta_deg)
            phi_counts = self.axis_phi_cfg.kinematics_deg_to_counts(ik.phi_deg)

            self._check_position_limits("theta", theta_counts)
            self._check_position_limits("phi", phi_counts)

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

    def _apply_position_limits(self, axis, cfg: AxisConfig):
        """Configure both EPOS-side and Python-side position limits for one
        axis after it has homed. Limits are derived from the homing direction
        and ``cfg.joint_travel_deg``.

        Positive homing → home is at the high end → allowed range [-travel, 0].
        Negative homing → home is at the low  end → allowed range [0, +travel].
        """
        direction = _homing_direction(cfg.home_method)
        if direction == 0:
            log.warning("axis %s: home_method %d has no clear direction; "
                        "skipping position limits", cfg.name, cfg.home_method)
            return
        # If we backed off from the hard stop and re-zeroed there, the
        # effective travel from the new home is the original mechanical
        # range minus the back-off amount.
        effective_travel_deg = max(cfg.joint_travel_deg - cfg.home_back_off_deg, 0.0)
        travel_counts = int(round(effective_travel_deg * cfg.counts_per_deg))
        margin_counts = int(round(cfg.home_overshoot_margin_deg * cfg.counts_per_deg))
        if direction > 0:
            lo, hi = -travel_counts, margin_counts
        else:
            lo, hi = -margin_counts, travel_counts
        try:
            axis.set_position_limits(lo, hi)
        except EposError as e:
            log.error("axis %s: setting EPOS position limits failed: %s", cfg.name, e)
            raise
        self._pos_limits[cfg.name] = (lo, hi)
        log.info("axis %s: position limits [%d, %d] counts (±%.1f° around home)",
                 cfg.name, lo, hi, cfg.joint_travel_deg)

    def _check_position_limits(self, axis_name: str, target_counts: int):
        limits = self._pos_limits.get(axis_name)
        if limits is None:
            return  # axis not homed yet
        lo, hi = limits
        if target_counts < lo or target_counts > hi:
            raise BoundsError(
                f"{axis_name} target {target_counts} counts outside "
                f"position limits [{lo}, {hi}]"
            )

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
        """Relative workspace move. Reads the current pose from the encoders
        each time so it works without a running state-poll thread."""
        theta_kdeg = self.axis_theta_cfg.counts_to_kinematics_deg(self.axis_theta.position())
        phi_kdeg = self.axis_phi_cfg.counts_to_kinematics_deg(self.axis_phi.position())
        x, y, _ = self.kin.forward_kinematics(
            math.radians(theta_kdeg), math.radians(phi_kdeg)
        )
        self.move_to(x + dx, y + dy, **kwargs)

    def rotate_both_axes(self, joint_deg: float, absolute: bool = False,
                         wait: bool = True, timeout_s: float = 60.0,
                         velocity_rpm: int | None = None):
        """Rotate both axes by (or to) the same joint angle simultaneously.
        Both target counts are computed and limit-checked before either
        motor is commanded, so a BoundsError aborts cleanly.

        ``velocity_rpm`` (motor-shaft RPM) overrides the per-axis default
        profile velocity just for this move; pass None to use config."""
        with self._action_lock:
            plan = []
            for name, axis, cfg in (
                ("theta", self.axis_theta, self.axis_theta_cfg),
                ("phi",   self.axis_phi,   self.axis_phi_cfg),
            ):
                command_counts = cfg.deg_to_counts(joint_deg)
                current = axis.position()
                target = command_counts if absolute else current + command_counts
                self._check_position_limits(name, target)
                plan.append((name, axis, cfg, command_counts, current, target))

            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                for name, axis, cfg, command_counts, current, target in plan:
                    v_rpm = velocity_rpm if velocity_rpm is not None else cfg.motor_velocity_rpm
                    axis.set_position_profile(
                        v_rpm,
                        cfg.motor_accel_rpm_s,
                        cfg.motor_accel_rpm_s,
                    )
                    log.info(
                        "rotate_both %s %s: current=%+d target=%+d (delta=%+d) @ %d rpm",
                        name, "abs" if absolute else "rel",
                        current, target, target - current, v_rpm,
                    )
                    axis.move_to(command_counts, absolute=absolute)
                if wait:
                    self._wait_done_with_current_guard(timeout_s=timeout_s)
            except (EposError, OvercurrentError) as e:
                self._set_state(fault=True, last_error=str(e))
                raise
            finally:
                self._set_state(busy=False)
                self._broadcast()

    def rotate_axis(self, name: str, joint_deg: float,
                    absolute: bool = False,
                    wait: bool = True, timeout_s: float = 60.0):
        """Move one joint to/by ``joint_deg``.

        ``absolute=False`` (default) → relative delta from the current pose.
        ``absolute=True``             → absolute joint angle (post-gear).

        Honors the same post-homing position limits and overcurrent guard
        that ``move_to`` uses.
        """
        if name == "theta":
            axis, cfg = self.axis_theta, self.axis_theta_cfg
        elif name == "phi":
            axis, cfg = self.axis_phi, self.axis_phi_cfg
        else:
            raise ValueError(f"unknown axis {name!r}; expected 'theta' or 'phi'")

        with self._action_lock:
            command_counts = cfg.deg_to_counts(joint_deg)
            current = axis.position()
            if absolute:
                target = command_counts
            else:
                target = current + command_counts
            self._check_position_limits(name, target)

            log.debug(
                "rotate_axis %s %s: joint=%+.3f° command_counts=%+d "
                "current=%+d target=%+d",
                name, "abs" if absolute else "rel",
                joint_deg, command_counts, current, target,
            )

            self._set_state(busy=True, last_error="")
            self._broadcast()
            try:
                # Re-apply the per-axis motor profile in case something
                # changed it (e.g. set_feed_mm_s from a prior workspace move).
                axis.set_position_profile(
                    cfg.motor_velocity_rpm,
                    cfg.motor_accel_rpm_s,
                    cfg.motor_accel_rpm_s,
                )
                axis.move_to(command_counts, absolute=absolute)
                if wait:
                    self._wait_done_with_current_guard(timeout_s=timeout_s)
            except (EposError, OvercurrentError) as e:
                self._set_state(fault=True, last_error=str(e))
                raise
            finally:
                self._set_state(busy=False)
                self._broadcast()

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
                theta_kdeg = theta_deg - self.axis_theta_cfg.home_offset_deg
                phi_kdeg = phi_deg - self.axis_phi_cfg.home_offset_deg
                x, y, z = self.kin.forward_kinematics(
                    math.radians(theta_kdeg), math.radians(phi_kdeg)
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
