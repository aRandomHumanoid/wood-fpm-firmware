"""Entrypoint: load config, wire up controllers, serve the web UI."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
from pathlib import Path

import yaml
from werkzeug.serving import make_server

from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import MotionController
from src.core.scan import ScanRunner
from src.web.app import create_app
from src.web.plot_viewer import create_plot_viewer_app
from src.web.scan_history import ScanHistoryCsvStore


class BackgroundServer:
    def __init__(self, host: str, port: int, app, name: str):
        self._server = make_server(host, port, app, threaded=True)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True, name=name)

    def start(self):
        self._thread.start()

    def shutdown(self):
        self._server.shutdown()
        self._thread.join(timeout=2.0)
        self._server.server_close()


def load_config(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_controller(cfg: dict) -> tuple[MotionController, ScanRunner]:
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

    scan = ScanRunner(motion=motion)
    motion.start_polling(hz=1.0)
    return motion, scan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml", type=Path)
    parser.add_argument("--host", default=None, help="override config.network.host")
    parser.add_argument("--port", default=None, type=int)
    parser.add_argument("--plot-port", default=None, type=int, help="override config.network.plot_port")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
    )

    cfg = load_config(args.config)
    motion, scan = build_controller(cfg)
    scan_history_path = args.config.resolve().parent / "scan_history.csv"
    scan_history = ScanHistoryCsvStore(scan_history_path)
    app, sio = create_app(
        motion=motion,
        scan=scan,
        scan_history_path=scan_history_path,
        scan_history_store=scan_history,
    )
    plot_app = create_plot_viewer_app(
        scan_history_path=scan_history_path,
        scan_history_store=scan_history,
    )

    host = args.host or cfg["network"]["host"]
    port = args.port or cfg["network"]["port"]
    plot_port = args.plot_port or cfg["network"].get("plot_port") or (port + 1)
    shutdown_started = False
    plot_server: BackgroundServer | None = None

    if plot_port == port:
        logging.warning("plot viewer disabled because plot_port matches main port: %s", plot_port)
    else:
        try:
            plot_server = BackgroundServer(host=host, port=plot_port, app=plot_app, name=f"plot-viewer-{plot_port}")
            plot_server.start()
            logging.info("plot viewer on http://%s:%d", host, plot_port)
        except OSError as e:
            logging.warning("plot viewer failed to bind on %s:%d: %s", host, plot_port, e)

    def shutdown():
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True
        logging.info("shutting down")
        if plot_server is not None:
            plot_server.shutdown()
        scan.abort()
        motion.shutdown()

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
