"""Entrypoint: load config, wire up controllers, serve the web UI."""

from __future__ import annotations

import argparse
import logging
import signal
from pathlib import Path

import yaml

from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import MotionController
from src.core.probe import ProbeMonitor
from src.core.scan import ScanRunner
from src.web.app import create_app


def load_config(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_controller(cfg: dict) -> tuple[MotionController, ProbeMonitor, ScanRunner]:
    m = cfg["machine"]
    bounds = MachineBounds(**m["bounds"])
    limits = MotionLimits(v_max_mm_s=m["v_max_mm_s"], a_max_mm_s2=m["a_max_mm_s2"])

    marlin_cfg = cfg.get("marlin", {})
    motion = MotionController(
        bounds=bounds,
        limits=limits,
        marlin_port=marlin_cfg.get("port", ""),
        marlin_baudrate=marlin_cfg.get("baudrate", 250000),
        x_axis=marlin_cfg.get("x_axis", "X"),
        y_axis=marlin_cfg.get("y_axis", "Y"),
        z_axis=marlin_cfg.get("z_axis", "Z"),
        simulate=cfg["driver"]["simulate"],
        auto_connect=cfg["driver"]["simulate"],
        rapid_feed_mm_s=m.get("rapid_feed_mm_s", 15.0),
        scan_feed_mm_s=m.get("scan_feed_mm_s", 2.0),
    )

    p = cfg["probe"]
    probe = ProbeMonitor(gpio_pin=p["gpio_pin"], active_low=p["active_low"], debounce_us=p["debounce_us"])

    scan = ScanRunner(motion=motion, probe=probe)
    motion.start_polling(hz=5.0)   # M114 round-trip is ~5-15ms over USB-CDC
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
    app, sio = create_app(
        motion=motion,
        probe=probe,
        scan=scan,
        scan_history_path=args.config.resolve().parent / "scan_history.csv",
    )

    host = args.host or cfg["network"]["host"]
    port = args.port or cfg["network"]["port"]
    shutdown_started = False

    def shutdown():
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True
        logging.info("shutting down")
        scan.abort()
        motion.shutdown()
        probe.close()

    def handle_signal(_signum, _frame):
        shutdown()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    logging.info("serving on http://%s:%d", host, port)
    try:
        sio.run(
            app,
            host=host,
            port=port,
            debug=args.debug,
            use_reloader=False,
            allow_unsafe_werkzeug=True,
        )
    finally:
        shutdown()


if __name__ == "__main__":
    main()
