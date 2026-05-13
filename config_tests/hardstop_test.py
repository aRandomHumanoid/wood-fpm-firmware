"""Hardstop homing test for motor 1 (theta) AND motor 2 (phi).

Spins each motor at a slow homing speed until the measured motor current
crosses a threshold (indicating the joint has hit a hard mechanical stop),
at which point the EPOS firmware halts, zeros the position, and reports
"homing attained". Motor 2 is homed only after motor 1 completes.

While homing is in progress this script polls position + motor current and
logs both at ~10 Hz so you can see what current the joint actually drew
before the threshold tripped.

Safety:
  - This *will* drive each joint into a physical stop. Verify there is one
    in the expected direction and nothing else in the way before running.
  - HOMING_DIRECTION below picks the direction:
      "positive" = HOMING_CURRENT_THRESHOLD_POS (-3)
      "negative" = HOMING_CURRENT_THRESHOLD_NEG (-4)
    Flip it if a motor heads the wrong way.
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
CURRENT_THRESHOLD_MA = 2000     # motor current that signals the hard stop
HOME_POSITION_COUNTS = 0        # position assigned to the home location
HOME_OFFSET_COUNTS = 0          # offset from the trip point to the zero
HOMING_TIMEOUT_S = 60.0
POLL_HZ = 10.0
INTER_AXIS_DELAY_S = 1.0        # pause between motor 1 and motor 2 homing
# ------------------------------------------------------------------------


def home_axis(motor: EposAxis, name: str, method: int) -> bool:
    """Run a single hardstop homing cycle on one axis. Returns True on success."""
    log = logging.getLogger(name)

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

    readback = motor.get_homing_parameter()
    log.info("homing params readback: %s", readback)

    log.info("starting homing search (method=%d)", method)
    t0 = time.monotonic()
    motor.start_homing()

    period = 1.0 / POLL_HZ
    peak_current = 0
    while True:
        elapsed = time.monotonic() - t0
        pos = motor.position()
        try:
            cur = motor.current_mA()
        except EposError:
            cur = 0
        peak_current = max(peak_current, abs(cur))
        attained, homing_err = motor.homing_state()
        sw = motor.statusword()
        fault = motor.fault_state()
        tag = ""
        if attained:
            tag = "  [HOMED]"
        elif homing_err:
            tag = "  [HOMING ERROR]"
        elif fault:
            tag = "  [FAULT]"
        elif sw & (1 << 11):
            tag = "  [INTERNAL LIMIT ACTIVE]"
        log.info("t=%.2fs  pos=%d  current=%+d mA  statusword=0x%04X%s",
                 elapsed, pos, cur, sw, tag)
        if attained:
            log.info("homing complete in %.2fs; peak |current| = %d mA; final pos = %d",
                     elapsed, peak_current, motor.position())
            return True
        if homing_err or fault:
            log.error("EPOS reported %s — stopping", "fault" if fault else "homing error")
            motor.stop_homing()
            return False
        if elapsed > HOMING_TIMEOUT_S:
            motor.stop_homing()
            log.error("homing timed out after %.1fs (peak |current|=%d mA)",
                      HOMING_TIMEOUT_S, peak_current)
            return False
        time.sleep(period)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-5s %(name)s: %(message)s")
    log = logging.getLogger("hardstop_test")

    cfg = yaml.safe_load(Path("config.yaml").read_text())
    theta = cfg["axes"]["theta"]
    phi = cfg["axes"]["phi"]
    driver_cfg = cfg["driver"]

    if driver_cfg.get("simulate"):
        log.warning("config has driver.simulate=true; this script is intended for real hardware")

    method = (
        HOMING_CURRENT_THRESHOLD_POS if HOMING_DIRECTION == "positive"
        else HOMING_CURRENT_THRESHOLD_NEG
    )

    log.info(
        "homing both motors: %s direction, %d rpm, %d mA threshold",
        HOMING_DIRECTION, HOMING_SPEED_RPM, CURRENT_THRESHOLD_MA,
    )
    log.info("theta: %s node %d   |   phi: %s node %d",
             theta["port_name"], theta["epos_node"],
             phi["port_name"], phi["epos_node"])

    driver_theta = EposDriver(library_path=driver_cfg["epos_library"], port_name=theta["port_name"])
    driver_phi = EposDriver(library_path=driver_cfg["epos_library"], port_name=phi["port_name"])

    motor1 = EposAxis(driver_theta, theta["epos_node"])
    motor2 = EposAxis(driver_phi, phi["epos_node"])

    rc = 0
    try:
        log.info("=== motor 1 (theta) ===")
        if not home_axis(motor1, "theta", method):
            log.error("motor 1 homing failed; skipping motor 2")
            return 1

        log.info("pausing %.1fs before motor 2", INTER_AXIS_DELAY_S)
        time.sleep(INTER_AXIS_DELAY_S)

        log.info("=== motor 2 (phi) ===")
        if not home_axis(motor2, "phi", method):
            log.error("motor 2 homing failed")
            rc = 1
        else:
            log.info("both motors homed")

    except EposError as e:
        log.error("EPOS error: %s", e)
        rc = 1
    except KeyboardInterrupt:
        log.warning("interrupted — halting both motors")
        for m in (motor1, motor2):
            try:
                m.halt()
            except EposError:
                pass
        rc = 130
    finally:
        for m, n in ((motor1, "theta"), (motor2, "phi")):
            try:
                m.disable()
                log.info("disabled %s", n)
            except EposError as e:
                log.warning("disable %s: %s", n, e)
        driver_theta.close()
        driver_phi.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
