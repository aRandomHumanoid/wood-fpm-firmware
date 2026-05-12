"""Hardware smoke test: enable both Maxon EPOS2 motors, spin each one
one OUTPUT-shaft revolution sequentially, then disable.

Each EPOS2 has its own USB cable to the Pi; Maxon enumerates them as
USB0, USB1, ... (the labels in config.yaml). This script opens one
EposDriver per port.

Counts math (per axis, from config.yaml):
    counts per motor rev = `counts_per_rev` (encoder cpt × 4 for the
        EPOS's 4x quadrature decoding — e.g. 1024 cpt → 4096 counts)
    counts per output rev = `counts_per_rev` × `gear_ratio`

Run on the Pi with the EPOS2 controllers powered and connected:
    python3 motor_test.py

This is NOT a pytest test — it talks to real hardware.
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

import yaml

from src.drivers.epos import EposDriver, EposAxis, EposError

TEST_VELOCITY_RPM = 200      # motor-shaft RPM during the test move
TEST_ACCEL_RPM_S = 1000      # motor-shaft accel/decel during the test move


def counts_per_output_rev(ax_cfg: dict) -> int:
    """Total EPOS counts for one full revolution of the output shaft."""
    return int(round(ax_cfg["counts_per_rev"] * ax_cfg["gear_ratio"]))


def spin(axis: EposAxis, name: str, counts: int):
    log = logging.getLogger(name)
    axis.activate_pp_mode()
    axis.set_position_profile(TEST_VELOCITY_RPM, TEST_ACCEL_RPM_S, TEST_ACCEL_RPM_S)

    start = axis.position()
    log.info("moving +%d counts (start=%d)", counts, start)
    axis.move_to(counts, absolute=False, immediately=True)
    axis.wait_done(timeout_s=30.0)
    log.info("done (pos=%d, delta=%d)", axis.position(), axis.position() - start)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-5s %(name)s: %(message)s")
    log = logging.getLogger("motor_test")

    cfg = yaml.safe_load(Path("config.yaml").read_text())
    axes_cfg = cfg["axes"]
    driver_cfg = cfg["driver"]

    if driver_cfg.get("simulate"):
        log.warning("config has driver.simulate=true; this script is intended for real hardware")

    theta = axes_cfg["theta"]
    phi = axes_cfg["phi"]

    log.info("opening EPOS on %s (theta) and %s (phi), lib=%s",
             theta["port_name"], phi["port_name"], driver_cfg["epos_library"])

    driver_theta = EposDriver(
        library_path=driver_cfg["epos_library"],
        port_name=theta["port_name"],
    )
    driver_phi = EposDriver(
        library_path=driver_cfg["epos_library"],
        port_name=phi["port_name"],
    )

    motor1 = EposAxis(driver_theta, theta["epos_node"])
    motor2 = EposAxis(driver_phi, phi["epos_node"])

    try:
        # Enable both first so we know both are live before spinning either.
        for ax, name in ((motor1, "theta"), (motor2, "phi")):
            ax.clear_fault()
            ax.enable()
            log.info("enabled %s (node %d)", name, ax.node_id.value)

        time.sleep(0.2)

        spin(motor1, "theta", counts_per_output_rev(theta))
        time.sleep(0.5)
        spin(motor2, "phi", counts_per_output_rev(phi))

    except EposError as e:
        log.error("EPOS error: %s", e)
        return 1
    finally:
        for ax, name in ((motor1, "theta"), (motor2, "phi")):
            try:
                ax.disable()
                log.info("disabled %s", name)
            except EposError as e:
                log.warning("disable %s: %s", name, e)
        driver_theta.close()
        driver_phi.close()

    log.info("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
