"""Touch-probe GPIO monitor.

Uses ``pigpio`` when available (sub-µs edge timing on the Pi). Falls back to a
software simulator (always returns False / never triggers) when not on hardware,
so the rest of the firmware can run unchanged in dev environments.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

log = logging.getLogger(__name__)


class ProbeMonitor:
    def __init__(self, gpio_pin: int, active_low: bool = True, debounce_us: int = 200):
        self.gpio_pin = gpio_pin
        self.active_low = active_low
        self.debounce_us = debounce_us
        self._listeners: list[Callable[[bool], None]] = []
        self._lock = threading.Lock()
        try:
            import pigpio  # type: ignore
            self._pi = pigpio.pi()
            if not self._pi.connected:
                raise RuntimeError("pigpiod not running")
            self._pi.set_mode(gpio_pin, pigpio.INPUT)
            self._pi.set_pull_up_down(gpio_pin, pigpio.PUD_UP if active_low else pigpio.PUD_DOWN)
            self._pi.set_glitch_filter(gpio_pin, debounce_us)
            self._cb = self._pi.callback(
                gpio_pin,
                pigpio.EITHER_EDGE,
                self._on_edge,
            )
            self._mode = "pigpio"
            log.info("ProbeMonitor pigpio on BCM%d (active_low=%s)", gpio_pin, active_low)
        except Exception as e:
            log.warning("Probe falling back to simulator: %s", e)
            self._pi = None
            self._cb = None
            self._mode = "sim"
            self._sim_triggered = False

    # -------- public API --------

    def is_triggered(self) -> bool:
        if self._mode == "pigpio":
            level = self._pi.read(self.gpio_pin)
            return (level == 0) if self.active_low else (level == 1)
        return self._sim_triggered

    def on_change(self, fn: Callable[[bool], None]):
        with self._lock:
            self._listeners.append(fn)

    def wait_for_trigger(self, timeout_s: float, poll_s: float = 0.001) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.is_triggered():
                return True
            time.sleep(poll_s)
        return False

    # -------- simulator hook --------

    def sim_set(self, triggered: bool):
        """Test/dev helper — directly set the simulated probe state."""
        if self._mode != "sim":
            return
        self._sim_triggered = bool(triggered)
        self._notify(self._sim_triggered)

    def close(self):
        if self._cb is not None:
            self._cb.cancel()
        if self._pi is not None:
            self._pi.stop()

    # -------- internals --------

    def _on_edge(self, _gpio, level, _tick):
        triggered = (level == 0) if self.active_low else (level == 1)
        self._notify(triggered)

    def _notify(self, triggered: bool):
        with self._lock:
            listeners = list(self._listeners)
        for fn in listeners:
            try:
                fn(triggered)
            except Exception:
                log.exception("probe listener raised")
