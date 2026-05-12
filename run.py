"""Entrypoint: load config, wire up controllers, serve the web UI."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
from pathlib import Path

import yaml

from src.core.kinematics import PPMKinematics
from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import AxisConfig, MotionController
from src.core.probe import ProbeMonitor
from src.core.scan import ScanRunner
from src.web.app import create_app


def load_config(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_controller(cfg: dict) -> tuple[MotionController, ProbeMonitor, ScanRunner]:
    m = cfg["machine"]
    bounds = MachineBounds(**m["bounds"])
    limits = MotionLimits(
        v_max_mm_s=m["v_max_mm_s"],
        a_max_mm_s2=m["a_max_mm_s2"],
        overcurrent_mA=m.get("overcurrent_limit_mA", 0),
    )

    k = cfg["kinematics"]
    kin = PPMKinematics(Lc=k["Lc"], H=k["H"], D=k["D"], G_deg=k["G"], theta_max_deg=k["theta_max_deg"])

    axes = cfg["axes"]
    ax_theta = AxisConfig(name="theta", **axes["theta"])
    ax_phi = AxisConfig(name="phi", **axes["phi"])

    motion = MotionController(
        kin=kin,
        bounds=bounds,
        limits=limits,
        axis_theta=ax_theta,
        axis_phi=ax_phi,
        simulate=cfg["driver"]["simulate"],
        epos_library=cfg["driver"]["epos_library"],
        rapid_feed_mm_s=m.get("rapid_feed_mm_s", 15.0),
        scan_feed_mm_s=m.get("scan_feed_mm_s", 2.0),
    )

    p = cfg["probe"]
    probe = ProbeMonitor(gpio_pin=p["gpio_pin"], active_low=p["active_low"], debounce_us=p["debounce_us"])

    scan = ScanRunner(motion=motion, probe=probe)
    motion.start_polling(hz=20.0)
    return motion, probe, scan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml", type=Path)
    parser.add_argument("--host", default=None, help="override config.network.host")
    parser.add_argument("--port", default=None, type=int)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
    )

    cfg = load_config(args.config)
    motion, probe, scan = build_controller(cfg)
    app, sio = create_app(motion=motion, probe=probe, scan=scan)

    host = args.host or cfg["network"]["host"]
    port = args.port or cfg["network"]["port"]

    def shutdown(*_):
        logging.info("shutting down")
        scan.abort()
        motion.shutdown()
        probe.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    logging.info("serving on http://%s:%d", host, port)
    sio.run(app, host=host, port=port, debug=args.debug, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
