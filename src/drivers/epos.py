"""Maxon EPOS2 driver — ctypes wrapper around libEposCmd.so.

One ``EposDriver`` instance owns one device handle (one USB connection to
one EPOS2 controller). Motors reached over that link are addressed by CAN
node id via ``EposAxis`` — typically a single node when each motor has its
own USB cable, or several nodes when one EPOS2 acts as a CAN gateway.

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
    c_byte,
    c_char_p,
    c_int,
    c_long,
    c_short,
    c_uint,
    c_ushort,
    c_void_p,
)
from typing import Optional

log = logging.getLogger(__name__)


# Homing method codes (EPOS2 Application Note "Device Programming";
# matches HM_* constants in /opt/EposCmdLib_*/include/Definitions.h)
HOMING_INDEX_POS = 34   # Index pulse, positive speed
HOMING_INDEX_NEG = 33   # Index pulse, negative speed
HOMING_CURRENT_THRESHOLD_POS = -3  # Hard-stop: positive speed, current threshold
HOMING_CURRENT_THRESHOLD_NEG = -4  # Hard-stop: negative speed, current threshold


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

        # Real signature (from EposCmdLib Definitions.h):
        #   VCS_SetHomingParameter(handle, nodeId,
        #       uint HomingAcceleration, uint SpeedSwitch, uint SpeedIndex,
        #       int  HomeOffset,   ushort CurrentThreshold,   int HomePosition,
        #       uint* pErrorCode)
        L.VCS_SetHomingParameter.restype = c_int
        L.VCS_SetHomingParameter.argtypes = [
            c_void_p, c_ushort,
            c_uint, c_uint, c_uint,
            c_int, c_ushort, c_int,
            POINTER(c_uint),
        ]

        # VCS_GetCurrentIsAveraged(handle, node, short* currentIs_mA, uint* err)
        L.VCS_GetCurrentIsAveraged.restype = c_int
        L.VCS_GetCurrentIsAveraged.argtypes = [c_void_p, c_ushort, POINTER(c_short), POINTER(c_uint)]

        # VCS_FindHome(handle, node, signed char method, uint* err)
        L.VCS_FindHome.restype = c_int
        L.VCS_FindHome.argtypes = [c_void_p, c_ushort, c_byte, POINTER(c_uint)]

        # VCS_GetHomingState(handle, node, int* attained, int* error, uint* err)
        L.VCS_GetHomingState.restype = c_int
        L.VCS_GetHomingState.argtypes = [c_void_p, c_ushort, POINTER(c_int), POINTER(c_int), POINTER(c_uint)]

        # VCS_GetHomingParameter(handle, node, uint* accel, uint* speedSwitch,
        #   uint* speedIndex, int* homeOffset, ushort* currentThreshold,
        #   int* homePosition, uint* err)
        L.VCS_GetHomingParameter.restype = c_int
        L.VCS_GetHomingParameter.argtypes = [
            c_void_p, c_ushort,
            POINTER(c_uint), POINTER(c_uint), POINTER(c_uint),
            POINTER(c_int), POINTER(c_ushort), POINTER(c_int),
            POINTER(c_uint),
        ]

        # VCS_StopHoming(handle, node, uint* err)
        L.VCS_StopHoming.restype = c_int
        L.VCS_StopHoming.argtypes = [c_void_p, c_ushort, POINTER(c_uint)]

        # VCS_GetObject(handle, node, idx, subidx, void* data, uint nbytes,
        #   uint* nbytesRead, uint* err)
        L.VCS_GetObject.restype = c_int
        L.VCS_GetObject.argtypes = [
            c_void_p, c_ushort, c_ushort, ctypes.c_ubyte,
            c_void_p, c_uint, POINTER(c_uint), POINTER(c_uint),
        ]

        # VCS_SetObject(handle, node, idx, subidx, void* data, uint nbytes,
        #   uint* nbytesWritten, uint* err)
        L.VCS_SetObject.restype = c_int
        L.VCS_SetObject.argtypes = [
            c_void_p, c_ushort, c_ushort, ctypes.c_ubyte,
            c_void_p, c_uint, POINTER(c_uint), POINTER(c_uint),
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
        current_threshold_mA: int = 0,
    ):
        """Configure the EPOS homing parameters.

        ``current_threshold_mA`` is only consulted by current-threshold
        (hard-stop) homing methods (-1, -2, -3, -4). For index-pulse methods
        (33, 34) leave it at 0.
        """
        err = c_uint(0)
        ok = self.drv._lib.VCS_SetHomingParameter(
            self.drv._handle,
            self.node_id,
            c_uint(int(acceleration_rpm_s)),
            c_uint(int(home_speed_rpm)),
            c_uint(int(zero_speed_rpm)),
            c_int(int(offset_counts)),
            c_ushort(int(current_threshold_mA)),
            c_int(int(position_counts)),
            byref(err),
        )
        if not ok:
            raise EposError("VCS_SetHomingParameter", err.value, self.drv._err_text(err.value))
        # Method is passed to VCS_FindHome rather than VCS_SetHomingParameter.
        self._home_method = method

    def start_homing(self):
        """Start homing using the previously-set method. Returns immediately;
        poll ``target_reached()`` (or call ``wait_done``) for completion."""
        method = getattr(self, "_home_method", HOMING_INDEX_POS)
        err = c_uint(0)
        ok = self.drv._lib.VCS_FindHome(self.drv._handle, self.node_id, c_byte(int(method)), byref(err))
        if not ok:
            raise EposError("VCS_FindHome", err.value, self.drv._err_text(err.value))

    def find_home(self, timeout_s: float = 60.0):
        self.start_homing()
        self.wait_done(timeout_s=timeout_s, poll_s=0.05)

    def current_mA(self) -> int:
        """Filtered motor current in mA (signed; sign indicates direction)."""
        cur = c_short(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetCurrentIsAveraged(self.drv._handle, self.node_id, byref(cur), byref(err))
        if not ok:
            raise EposError("VCS_GetCurrentIsAveraged", err.value, self.drv._err_text(err.value))
        return int(cur.value)

    def homing_state(self) -> tuple[bool, bool]:
        """Return (attained, error) for the current/most-recent homing run."""
        attained = c_int(0)
        homing_err = c_int(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetHomingState(
            self.drv._handle, self.node_id, byref(attained), byref(homing_err), byref(err)
        )
        if not ok:
            raise EposError("VCS_GetHomingState", err.value, self.drv._err_text(err.value))
        return bool(attained.value), bool(homing_err.value)

    def get_homing_parameter(self) -> dict:
        """Read back the EPOS's current homing parameters. Useful for verifying
        a prior ``set_homing_parameter`` call actually took effect."""
        accel = c_uint(0)
        speed_switch = c_uint(0)
        speed_index = c_uint(0)
        offset = c_int(0)
        threshold = c_ushort(0)
        position = c_int(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetHomingParameter(
            self.drv._handle, self.node_id,
            byref(accel), byref(speed_switch), byref(speed_index),
            byref(offset), byref(threshold), byref(position),
            byref(err),
        )
        if not ok:
            raise EposError("VCS_GetHomingParameter", err.value, self.drv._err_text(err.value))
        return dict(
            acceleration_rpm_s=accel.value,
            home_speed_rpm=speed_switch.value,
            zero_speed_rpm=speed_index.value,
            offset_counts=offset.value,
            current_threshold_mA=threshold.value,
            position_counts=position.value,
        )

    def stop_homing(self):
        err = c_uint(0)
        ok = self.drv._lib.VCS_StopHoming(self.drv._handle, self.node_id, byref(err))
        if not ok:
            raise EposError("VCS_StopHoming", err.value, self.drv._err_text(err.value))

    def set_position_limits(self, min_counts: int, max_counts: int):
        """Configure the EPOS software position limits (object 0x607D).
        Any subsequent move outside [min_counts, max_counts] is refused
        by the firmware (statusword bit 11 'Internal limit active' is set)."""
        for sub, value, label in (
            (0x01, min_counts, "min"),
            (0x02, max_counts, "max"),
        ):
            data = c_int(int(value))
            nbytes = c_uint(0)
            err = c_uint(0)
            ok = self.drv._lib.VCS_SetObject(
                self.drv._handle, self.node_id,
                c_ushort(0x607D), ctypes.c_ubyte(sub),
                byref(data), c_uint(4), byref(nbytes), byref(err),
            )
            if not ok:
                raise EposError(
                    f"VCS_SetObject(0x607D:{sub:02X} {label})",
                    err.value, self.drv._err_text(err.value),
                )

    def statusword(self) -> int:
        """Raw CiA 402 statusword (object 0x6041). Useful bits:
            bit  0..3: state machine
            bit  7: Warning
            bit 11: Internal limit active (software position limits)
            bit 12: Op-mode specific (homing mode: Homing attained)
            bit 13: Op-mode specific (homing mode: Homing error)
        """
        data = c_ushort(0)
        nbytes = c_uint(0)
        err = c_uint(0)
        ok = self.drv._lib.VCS_GetObject(
            self.drv._handle, self.node_id,
            c_ushort(0x6041), ctypes.c_ubyte(0x00),
            byref(data), c_uint(2), byref(nbytes), byref(err),
        )
        if not ok:
            raise EposError("VCS_GetObject(0x6041)", err.value, self.drv._err_text(err.value))
        return int(data.value)


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

    def start_homing(self):
        self._pos = 0
        self._target = 0
        self._move_started = None
        self._homed = True

    def current_mA(self) -> int:
        return 0

    def homing_state(self) -> tuple[bool, bool]:
        return getattr(self, "_homed", False), False

    def get_homing_parameter(self) -> dict:
        return dict(
            acceleration_rpm_s=500, home_speed_rpm=50, zero_speed_rpm=10,
            offset_counts=0, current_threshold_mA=0, position_counts=0,
        )

    def stop_homing(self):
        self._move_started = None

    def set_position_limits(self, min_counts: int, max_counts: int):
        self._pos_min, self._pos_max = int(min_counts), int(max_counts)

    def statusword(self) -> int:
        return 0x1237 if getattr(self, "_homed", False) else 0x0237  # bit 12 set if homed

    def move_to(self, target_counts: int, absolute: bool = True, immediately: bool = True):
        with self.drv._lock:
            self._start_pos = self._pos
            self._target = int(target_counts) if absolute else self._pos + int(target_counts)
            # crude time estimate: counts / (rev/s × counts_per_rev). Assume 4096
            # EPOS counts per motor rev (1024 cpt × 4x quadrature).
            distance = abs(self._target - self._start_pos)
            cps = max(self._vel_rpm / 60.0 * 4096.0, 1.0)
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
