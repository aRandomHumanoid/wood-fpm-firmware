"""Maxon EPOS2 driver — ctypes wrapper around libEposCmd.so.

One ``EposDriver`` instance owns the device handle (one USB/serial connection
to the gateway). Each motor on the bus is addressed by its CAN node id via
``EposAxis``.

A ``SimulatedEposDriver`` mirrors the same interface for off-hardware
development and unit tests.
"""

from __future__ import annotations

import ctypes
import logging
import threading
import time
from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_int,
    c_long,
    c_uint,
    c_ushort,
    c_void_p,
)
from typing import Optional

log = logging.getLogger(__name__)


# Homing method codes (EPOS2 Application Note "Device Programming")
HOMING_INDEX_POS = 34   # Index pulse, positive speed
HOMING_INDEX_NEG = 33   # Index pulse, negative speed


class EposError(RuntimeError):
    def __init__(self, op: str, code: int, message: str = ""):
        super().__init__(f"{op}: 0x{code:08X} {message}".strip())
        self.op = op
        self.code = code


class EposDriver:
    """Wraps libEposCmd.so. Owns one device handle."""

    def __init__(
        self,
        library_path: str = "libEposCmd.so.6.6.1.0",
        device_name: str = "EPOS2",
        protocol: str = "MAXON SERIAL V2",
        interface: str = "USB",
        port_name: str = "USB0",
    ):
        self._lib = ctypes.CDLL(library_path)
        self._configure_signatures()
        err = c_uint(0)
        handle = self._lib.VCS_OpenDevice(
            device_name.encode(),
            protocol.encode(),
            interface.encode(),
            port_name.encode(),
            byref(err),
        )
        if not handle:
            raise EposError("VCS_OpenDevice", err.value, self._err_text(err.value))
        self._handle = c_void_p(handle)
        self._lock = threading.Lock()
        log.info("Opened EPOS device %s @ %s (handle=%s)", device_name, port_name, handle)

    def _configure_signatures(self):
        L = self._lib
        L.VCS_OpenDevice.restype = c_void_p
        L.VCS_OpenDevice.argtypes = [c_char_p, c_char_p, c_char_p, c_char_p, POINTER(c_uint)]
        L.VCS_CloseDevice.restype = c_int
        L.VCS_CloseDevice.argtypes = [c_void_p, POINTER(c_uint)]
        L.VCS_GetErrorInfo.restype = c_int
        L.VCS_GetErrorInfo.argtypes = [c_uint, c_char_p, c_ushort]

        for name in (
            "VCS_SetEnableState",
            "VCS_SetDisableState",
            "VCS_ClearFault",
            "VCS_ActivateProfilePositionMode",
            "VCS_ActivateHomingMode",
            "VCS_HaltPositionMovement",
            "VCS_FindHome",
        ):
            fn = getattr(L, name)
            fn.restype = c_int
            fn.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]

        L.VCS_GetFaultState.restype = c_int
        L.VCS_GetFaultState.argtypes = [c_void_p, c_ushort, POINTER(c_int), POINTER(c_uint)]

        L.VCS_GetPositionIs.restype = c_int
        L.VCS_GetPositionIs.argtypes = [c_void_p, c_ushort, POINTER(c_long), POINTER(c_uint)]

        L.VCS_GetMovementState.restype = c_int
        L.VCS_GetMovementState.argtypes = [c_void_p, c_ushort, POINTER(c_int), POINTER(c_uint)]

        L.VCS_SetPositionProfile.restype = c_int
        L.VCS_SetPositionProfile.argtypes = [c_void_p, c_ushort, c_uint, c_uint, c_uint, POINTER(c_uint)]

        L.VCS_MoveToPosition.restype = c_int
        L.VCS_MoveToPosition.argtypes = [c_void_p, c_ushort, c_long, c_int, c_int, POINTER(c_uint)]

        L.VCS_SetHomingParameter.restype = c_int
        L.VCS_SetHomingParameter.argtypes = [
            c_void_p, c_ushort,
            c_uint, c_uint, c_uint, c_uint, c_long, c_long,
            POINTER(c_uint),
        ]

    def _err_text(self, code: int) -> str:
        buf = ctypes.create_string_buffer(256)
        self._lib.VCS_GetErrorInfo(code, buf, 256)
        return buf.value.decode(errors="replace")

    def _call(self, name: str, fn, *args):
        err = c_uint(0)
        with self._lock:
            ok = fn(self._handle, *args, byref(err))
        if not ok:
            raise EposError(name, err.value, self._err_text(err.value))

    def close(self):
        if self._handle:
            err = c_uint(0)
            self._lib.VCS_CloseDevice(self._handle, byref(err))
            self._handle = None


class EposAxis:
    def __init__(self, driver: EposDriver, node_id: int):
        self.drv = driver
        self.node_id = c_ushort(node_id)

    # ----- state -----
    def clear_fault(self):
        self.drv._call("VCS_ClearFault", self.drv._lib.VCS_ClearFault, self.node_id)

    def enable(self):
        self.drv._call("VCS_SetEnableState", self.drv._lib.VCS_SetEnableState, self.node_id)

    def disable(self):
        self.drv._call("VCS_SetDisableState", self.drv._lib.VCS_SetDisableState, self.node_id)

    def fault_state(self) -> bool:
        is_fault = c_int(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetFaultState(self.drv._handle, self.node_id, byref(is_fault), byref(err))
        if not ok:
            raise EposError("VCS_GetFaultState", err.value, self.drv._err_text(err.value))
        return bool(is_fault.value)

    # ----- profile position -----
    def activate_pp_mode(self):
        self.drv._call(
            "VCS_ActivateProfilePositionMode",
            self.drv._lib.VCS_ActivateProfilePositionMode,
            self.node_id,
        )

    def set_position_profile(self, velocity_rpm: int, accel_rpm_s: int, decel_rpm_s: int):
        err = c_uint(0)
        ok = self.drv._lib.VCS_SetPositionProfile(
            self.drv._handle,
            self.node_id,
            c_uint(int(velocity_rpm)),
            c_uint(int(accel_rpm_s)),
            c_uint(int(decel_rpm_s)),
            byref(err),
        )
        if not ok:
            raise EposError("VCS_SetPositionProfile", err.value, self.drv._err_text(err.value))

    def move_to(self, target_counts: int, absolute: bool = True, immediately: bool = True):
        err = c_uint(0)
        ok = self.drv._lib.VCS_MoveToPosition(
            self.drv._handle,
            self.node_id,
            c_long(int(target_counts)),
            c_int(1 if absolute else 0),
            c_int(1 if immediately else 0),
            byref(err),
        )
        if not ok:
            raise EposError("VCS_MoveToPosition", err.value, self.drv._err_text(err.value))

    def halt(self):
        self.drv._call(
            "VCS_HaltPositionMovement",
            self.drv._lib.VCS_HaltPositionMovement,
            self.node_id,
        )

    # ----- feedback -----
    def position(self) -> int:
        pos = c_long(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetPositionIs(self.drv._handle, self.node_id, byref(pos), byref(err))
        if not ok:
            raise EposError("VCS_GetPositionIs", err.value, self.drv._err_text(err.value))
        return int(pos.value)

    def target_reached(self) -> bool:
        state = c_int(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetMovementState(self.drv._handle, self.node_id, byref(state), byref(err))
        if not ok:
            raise EposError("VCS_GetMovementState", err.value, self.drv._err_text(err.value))
        return bool(state.value)

    def wait_done(self, timeout_s: float = 30.0, poll_s: float = 0.02):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.target_reached():
                return
            time.sleep(poll_s)
        raise EposError("wait_done", 0, "timeout waiting for target_reached")

    # ----- homing -----
    def activate_homing_mode(self):
        self.drv._call(
            "VCS_ActivateHomingMode",
            self.drv._lib.VCS_ActivateHomingMode,
            self.node_id,
        )

    def set_homing_parameter(
        self,
        method: int,
        home_speed_rpm: int,
        zero_speed_rpm: int = 10,
        acceleration_rpm_s: int = 500,
        offset_counts: int = 0,
        position_counts: int = 0,
    ):
        err = c_uint(0)
        ok = self.drv._lib.VCS_SetHomingParameter(
            self.drv._handle,
            self.node_id,
            c_uint(int(acceleration_rpm_s)),
            c_uint(int(home_speed_rpm)),
            c_uint(int(zero_speed_rpm)),
            c_uint(0),                # currentThreshold (mA) — not used for index homing
            c_long(int(offset_counts)),
            c_long(int(position_counts)),
            byref(err),
        )
        if not ok:
            raise EposError("VCS_SetHomingParameter", err.value, self.drv._err_text(err.value))
        # Method is set via separate object-dictionary call in some lib versions;
        # current libEposCmd exposes VCS_FindHome(method, ...) — pass it there instead.
        self._home_method = method

    def find_home(self, timeout_s: float = 60.0):
        method = getattr(self, "_home_method", HOMING_INDEX_POS)
        # VCS_FindHome signature varies; the v6 library accepts (handle, node, method, *err).
        fn = self.drv._lib.VCS_FindHome
        fn.argtypes = [c_void_p, c_ushort, c_int, POINTER(c_uint)]
        fn.restype = c_int
        err = c_uint(0)
        ok = fn(self.drv._handle, self.node_id, c_int(int(method)), byref(err))
        if not ok:
            raise EposError("VCS_FindHome", err.value, self.drv._err_text(err.value))
        self.wait_done(timeout_s=timeout_s, poll_s=0.05)


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------


class SimulatedEposDriver:
    """Software-only stand-in. Tracks a virtual position per node id."""

    def __init__(self, **_kwargs):
        self._lock = threading.Lock()
        self._handle = "sim"


class SimulatedEposAxis:
    def __init__(self, driver: SimulatedEposDriver, node_id: int):
        self.drv = driver
        self.node_id = node_id
        self._pos = 0
        self._target = 0
        self._enabled = False
        self._fault = False
        self._home_method = HOMING_INDEX_POS
        self._vel_rpm = 100
        self._move_started = None
        self._move_duration = 0.0
        self._start_pos = 0

    def clear_fault(self): self._fault = False
    def enable(self): self._enabled = True
    def disable(self): self._enabled = False
    def fault_state(self) -> bool: return self._fault
    def activate_pp_mode(self): pass
    def activate_homing_mode(self): pass

    def set_position_profile(self, velocity_rpm: int, accel_rpm_s: int, decel_rpm_s: int):
        self._vel_rpm = max(int(velocity_rpm), 1)

    def set_homing_parameter(self, method: int, **_):
        self._home_method = method

    def move_to(self, target_counts: int, absolute: bool = True, immediately: bool = True):
        with self.drv._lock:
            self._start_pos = self._pos
            self._target = int(target_counts) if absolute else self._pos + int(target_counts)
            # crude time estimate: counts / (rev/s * counts_per_rev). assume 1024 cpr.
            distance = abs(self._target - self._start_pos)
            cps = max(self._vel_rpm / 60.0 * 1024.0, 1.0)
            self._move_duration = distance / cps
            self._move_started = time.monotonic()

    def halt(self):
        self.position()  # update simulated position
        self._target = self._pos
        self._move_started = None

    def position(self) -> int:
        if self._move_started is None or self._move_duration == 0:
            return self._pos
        elapsed = time.monotonic() - self._move_started
        if elapsed >= self._move_duration:
            self._pos = self._target
            self._move_started = None
        else:
            frac = elapsed / self._move_duration
            self._pos = int(self._start_pos + frac * (self._target - self._start_pos))
        return self._pos

    def target_reached(self) -> bool:
        self.position()
        return self._move_started is None

    def wait_done(self, timeout_s: float = 30.0, poll_s: float = 0.02):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.target_reached():
                return
            time.sleep(poll_s)
        raise EposError("wait_done", 0, "sim timeout")

    def find_home(self, timeout_s: float = 60.0):
        self._pos = 0
        self._target = 0
        self._move_started = None


def make_driver(simulate: bool, **kwargs):
    if simulate:
        return SimulatedEposDriver(**kwargs)
    return EposDriver(**kwargs)


def make_axis(driver, node_id: int):
    if isinstance(driver, SimulatedEposDriver):
        return SimulatedEposAxis(driver, node_id)
    return EposAxis(driver, node_id)
