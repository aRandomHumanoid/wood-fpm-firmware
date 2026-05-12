"""Hardstop homing test for motor 1 (theta).

Spins the motor at a slow homing speed until the measured motor current
crosses a threshold (indicating the joint has hit a hard mechanical stop),
at which point the EPOS firmware halts, zeros the position, and reports
"homing attained".

While homing is in progress this script polls position + motor current and
logs both at ~10 Hz so you can see what current the joint actually drew
before the threshold tripped.

Safety:
  - This *will* drive the joint into a physical stop. Verify there is one
    in the expected direction and nothing else in the way before running.
  - HOMING_DIRECTION below picks the direction:
      "positive" = HOMING_CURRENT_THRESHOLD_POS (-3)
      "negative" = HOMING_CURRENT_THRESHOLD_NEG (-4)
    Flip it if the motor heads the wrong way.
  - CURRENT_THRESHOLD_MA is conservative; raise it only if the joint
    can't develop enough current to trip the threshold under nominal
    contact force.

Run:
    python3 hardstop_test.py
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

import yaml

from src.drivers.epos import (
    EposDriver,
    EposAxis,
    EposError,
    HOMING_CURRENT_THRESHOLD_POS,
    HOMING_CURRENT_THRESHOLD_NEG,
)

# --- tuning knobs --------------------------------------------------------
HOMING_DIRECTION = "negative"   # "positive" or "negative"
HOMING_SPEED_RPM = 50           # motor-shaft RPM during the search
HOMING_ACCEL_RPM_S = 500        # motor-shaft accel/decel
CURRENT_THRESHOLD_MA = 500      # motor current that signals the hard stop
HOME_POSITION_COUNTS = 0        # position assigned to the home location
HOME_OFFSET_COUNTS = 0          # offset from the trip point to the zero
HOMING_TIMEOUT_S = 60.0
POLL_HZ = 10.0
# ------------------------------------------------------------------------


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-5s %(name)s: %(message)s")
    log = logging.getLogger("hardstop_test")

    cfg = yaml.safe_load(Path("config.yaml").read_text())
    theta = cfg["axes"]["theta"]
    driver_cfg = cfg["driver"]

    if driver_cfg.get("simulate"):
        log.warning("config has driver.simulate=true; this script is intended for real hardware")

    method = (
        HOMING_CURRENT_THRESHOLD_POS if HOMING_DIRECTION == "positive"
        else HOMING_CURRENT_THRESHOLD_NEG
    )

    log.info(
        "homing motor 1 (theta) on %s, node %d: %s direction, %d rpm, %d mA threshold",
        theta["port_name"], theta["epos_node"],
        HOMING_DIRECTION, HOMING_SPEED_RPM, CURRENT_THRESHOLD_MA,
    )

    driver = EposDriver(library_path=driver_cfg["epos_library"], port_name=theta["port_name"])
    motor = EposAxis(driver, theta["epos_node"])

    try:
        motor.clear_fault()
        motor.enable()
        log.info("enabled")

        motor.activate_homing_mode()
        motor.set_homing_parameter(
            method=method,
            home_speed_rpm=HOMING_SPEED_RPM,
            acceleration_rpm_s=HOMING_ACCEL_RPM_S,
            offset_counts=HOME_OFFSET_COUNTS,
            position_counts=HOME_POSITION_COUNTS,
            current_threshold_mA=CURRENT_THRESHOLD_MA,
        )

        log.info("starting homing search")
        t0 = time.monotonic()
        motor.start_homing()

        # Poll until homing completes or timeout. The EPOS firmware decides
        # internally when the current threshold has been crossed; we just
        # log position + current while it works.
        period = 1.0 / POLL_HZ
        peak_current = 0
        while True:
            elapsed = time.monotonic() - t0
            pos = motor.position()
            try:
                cur = motor.current_mA()
            except EposError:
                cur = 0  # not fatal — keep polling state
            peak_current = max(peak_current, abs(cur))
            done = motor.target_reached()
            log.info("t=%.2fs  pos=%d counts  current=%+d mA%s",
                     elapsed, pos, cur, "  [HOMED]" if done else "")
            if done:
                break
            if elapsed > HOMING_TIMEOUT_S:
                motor.halt()
                log.error("homing timed out after %.1fs", HOMING_TIMEOUT_S)
                return 1
            time.sleep(period)

        log.info("homing complete in %.2fs; peak |current| = %d mA; final pos = %d",
                 elapsed, peak_current, motor.position())

    except EposError as e:
        log.error("EPOS error: %s", e)
        return 1
    except KeyboardInterrupt:
        log.warning("interrupted — halting motor")
        try:
            motor.halt()
        except EposError:
            pass
        return 130
    finally:
        try:
            motor.disable()
            log.info("disabled")
        except EposError as e:
            log.warning("disable: %s", e)
        driver.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
