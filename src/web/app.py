"""Flask + Socket.IO web layer.

Endpoints:
  GET  /             — single-page UI
  POST /api/home     — kick off homing on both axes
  POST /api/jog      — {dx, dy} or {x, y}
    POST /api/scan     — {x_max, y_max, y_min, n_samples}
  POST /api/scan/abort
  POST /api/stop     — E-stop
    POST /api/stop/reset
  GET  /api/limits   — read-only vel/accel/bounds
    GET  /api/scan/history.csv
    POST /api/scan/history/clear

Socket.IO events out:
  state         (~20 Hz)  — current pose / probe / busy / fault
    scan_started, scan_point, scan_complete  (used to trigger CSV-backed redraws)
"""

from __future__ import annotations

import logging
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Dict

from flask import Flask, Response, jsonify, render_template, request
from flask_socketio import SocketIO

from ..core.limits import BoundsError
from ..core.motion import MotionController
from ..core.probe import ProbeMonitor
from ..core.scan import PROBE_TARGET_X, ScanRequest, ScanRunner
from ..drivers.marlin import MarlinError
from .serial_console import SerialConsoleBuffer
from .scan_history import ScanHistoryCsvStore

log = logging.getLogger(__name__)


def discover_serial_ports(preferred_port: str | None = None) -> list[str]:
    ports = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports.extend(sorted(str(path) for path in Path("/dev").glob(pattern.removeprefix("/dev/"))))
    deduped = list(dict.fromkeys(ports))
    if preferred_port and preferred_port not in deduped:
        deduped.insert(0, preferred_port)
    return deduped


def create_app(
    motion: MotionController,
    probe: ProbeMonitor,
    scan: ScanRunner,
    scan_history_path: Path | str = Path("scan_history.csv"),
) -> tuple[Flask, SocketIO]:
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
    )
    scan_history = ScanHistoryCsvStore(Path(scan_history_path))
    serial_console = SerialConsoleBuffer()
    app.config["motion"] = motion
    app.config["probe"] = probe
    app.config["scan"] = scan
    app.config["scan_history"] = scan_history
    app.config["serial_console"] = serial_console

    socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
    active_scans: dict[str, ScanRequest] = {}

    def serial_payload() -> Dict[str, object]:
        status = motion.serial_status()
        return {
            **status,
            "ports": discover_serial_ports(str(status.get("port") or "")),
        }

    # --- streaming wiring ---
    def push_state(state):
        socketio.emit("state", state.to_dict())

    motion.subscribe(push_state)

    def push_serial(entry: Dict[str, object]):
        serial_console.append(entry)
        socketio.emit("serial_entry", entry)

    if hasattr(motion, "subscribe_serial"):
        motion.subscribe_serial(push_serial)

    def on_probe(triggered: bool):
        motion.set_probe_state(triggered)

    probe.on_change(on_probe)

    def on_started(req: ScanRequest):
        active_scans[req.scan_id] = req
        socketio.emit("scan_started", req.to_dict())

    def on_point(pt):
        req = active_scans.get(pt.scan_id)
        if req is not None:
            scan_history.append_point(req, pt)
        socketio.emit("scan_point", pt.to_dict())

    def on_complete(scan_id: str):
        active_scans.pop(scan_id, None)
        socketio.emit("scan_complete", {"scan_id": scan_id})

    scan.on_started = on_started
    scan.on_point = on_point
    scan.on_complete = on_complete

    # --- routes ---
    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/api/limits")
    def get_limits():
        return jsonify(
            {
                "bounds": {
                    "x_min": motion.bounds.x_min,
                    "x_max": motion.bounds.x_max,
                    "y_min": motion.bounds.y_min,
                    "y_max": motion.bounds.y_max,
                },
                "v_max_mm_s": motion.limits.v_max_mm_s,
                "a_max_mm_s2": motion.limits.a_max_mm_s2,
                "rapid_feed_mm_s": motion.rapid_feed_mm_s,
                "scan_feed_mm_s": motion.scan_feed_mm_s,
            }
        )

    @app.get("/api/serial")
    def get_serial_status():
        return jsonify(serial_payload())

    @app.post("/api/serial/connect")
    def post_serial_connect():
        data = request.get_json(force=True) or {}
        if scan.running or motion.is_busy():
            return jsonify({"ok": False, "error": "controller busy"}), 409
        try:
            status = motion.connect_serial(
                port=str(data.get("port") or motion.serial_status()["port"]),
                baudrate=int(data.get("baudrate") or motion.serial_status()["baudrate"]),
                simulate=(motion.serial_status()["simulate"] if "simulate" not in data else bool(data.get("simulate"))),
            )
        except (ValueError, MarlinError) as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        return jsonify({"ok": True, **serial_payload()})

    @app.post("/api/serial/disconnect")
    def post_serial_disconnect():
        if scan.running or motion.is_busy():
            return jsonify({"ok": False, "error": "controller busy"}), 409
        motion.disconnect_serial()
        return jsonify({"ok": True, **serial_payload()})

    @app.get("/api/serial/console")
    def get_serial_console():
        return jsonify({"entries": serial_console.snapshot()})

    @app.post("/api/serial/console/clear")
    def post_serial_console_clear():
        serial_console.clear()
        return jsonify({"ok": True})

    @app.post("/api/serial/command")
    def post_serial_command():
        data = request.get_json(force=True) or {}
        try:
            lines = motion.send_gcode(str(data.get("command", "")))
        except (ValueError, MarlinError) as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        return jsonify({"ok": True, "lines": lines})

    @app.post("/api/home")
    def post_home():
        if not motion.serial_status()["connected"]:
            return jsonify({"ok": False, "error": "serial not connected"}), 409
        threading.Thread(target=motion.home_all, daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/jog")
    def post_jog():
        data: Dict[str, Any] = request.get_json(force=True) or {}
        try:
            if not motion.serial_status()["connected"]:
                return jsonify({"ok": False, "error": "serial not connected"}), 409
            if "x" in data and "y" in data:
                x, y = float(data["x"]), float(data["y"])
                motion.bounds.check(x, y)
                threading.Thread(
                    target=motion.move_to, args=(x, y), daemon=True
                ).start()
            else:
                dx, dy = float(data.get("dx", 0)), float(data.get("dy", 0))
                # Pre-check against the last-polled position so obvious
                # out-of-range jogs return a clean 400 instead of failing
                # silently in the worker thread. The state snapshot is up to
                # ~200 ms stale (5 Hz polling) — borderline jogs still
                # re-check against Marlin's fresh position inside motion.jog.
                motion.bounds.check(motion.state.x + dx, motion.state.y + dy)
                threading.Thread(
                    target=motion.jog, args=(dx, dy), daemon=True
                ).start()
            return jsonify({"ok": True})
        except (BoundsError, ValueError) as e:
            return jsonify({"ok": False, "error": str(e)}), 400

    @app.post("/api/stop")
    def post_stop():
        scan.abort()
        motion.stop()
        return jsonify({"ok": True})

    @app.post("/api/stop/reset")
    def post_stop_reset():
        return jsonify({"ok": True, "reset": motion.reset_estop()})

    @app.post("/api/scan")
    def post_scan():
        data = request.get_json(force=True) or {}
        if not motion.serial_status()["connected"]:
            return jsonify({"ok": False, "error": "serial not connected"}), 409
        try:
            req = ScanRequest(
                x_max=float(data["x_max"]),
                y_max=float(data["y_max"]),
                y_min=float(data["y_min"]),
                n_samples=int(data["n_samples"]),
                scan_id=uuid.uuid4().hex[:8],
            )
        except (KeyError, ValueError) as e:
            return jsonify({"ok": False, "error": f"bad request: {e}"}), 400

        for x in (PROBE_TARGET_X, req.x_max):
            for y in (req.y_min, req.y_max):
                if not motion.bounds.contains(x, y):
                    return (
                        jsonify({"ok": False, "error": f"({x},{y}) outside workspace"}),
                        400,
                    )

        try:
            scan.start(req)
        except RuntimeError as e:
            return jsonify({"ok": False, "error": str(e)}), 409
        return jsonify({"ok": True, "scan_id": req.scan_id})

    @app.post("/api/scan/abort")
    def post_scan_abort():
        scan.abort()
        return jsonify({"ok": True})

    @app.get("/api/scan/history.csv")
    def get_scan_history_csv():
        return Response(scan_history.read_text(), mimetype="text/csv")

    @app.post("/api/scan/history/clear")
    def post_scan_history_clear():
        if scan.running:
            return jsonify({"ok": False, "error": "scan already running"}), 409
        scan_history.clear()
        return jsonify({"ok": True})

    return app, socketio
