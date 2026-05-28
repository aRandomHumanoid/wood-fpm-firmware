"""Flask + Socket.IO web layer.

Endpoints:
  GET  /             — single-page UI
  POST /api/home     — kick off homing on both axes
  POST /api/jog      — {dx, dy} or {x, y}
    POST /api/scan     — {x_max, probe_target_x, probe_speed_mm_s, y_max, y_min, n_samples}
  POST /api/scan/abort
  POST /api/stop     — E-stop
    POST /api/stop/reset
  GET  /api/limits   — read-only vel/accel/bounds
    GET  /api/scan/history.csv
    POST /api/scan/history/clear

Socket.IO events out:
    state         (~20 Hz)  — current pose / busy / fault
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
from ..core.scan import DEFAULT_PROBE_TARGET_X, ScanRequest, ScanRunner
from ..drivers.marlin import MarlinError
from .serial_console import SerialConsoleBuffer
from .scan_history import ScanHistoryCsvStore

log = logging.getLogger(__name__)

STATE_HEARTBEAT_S = 1.0


def discover_serial_ports(preferred_port: str | None = None) -> list[str]:
    ports = []

    try:
        from serial.tools import list_ports

        ports.extend(port.device for port in list_ports.comports() if port.device)
    except Exception:
        log.exception("serial port discovery failed")

    ports.extend(sorted(str(path) for path in Path("/dev").glob("pts/[0-9]*")))

    deduped = list(dict.fromkeys(ports))
    if preferred_port and preferred_port not in deduped:
        deduped.insert(0, preferred_port)
    return deduped


def create_app(
    motion: MotionController,
    scan: ScanRunner,
    scan_history_path: Path | str = Path("scan_history.csv"),
    scan_history_store: ScanHistoryCsvStore | None = None,
) -> tuple[Flask, SocketIO]:
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
    )
    scan_history = scan_history_store or ScanHistoryCsvStore(Path(scan_history_path))
    serial_console = SerialConsoleBuffer()
    app.config["motion"] = motion
    app.config["scan"] = scan
    app.config["scan_history"] = scan_history
    app.config["serial_console"] = serial_console

    socketio = SocketIO(
        app,
        cors_allowed_origins="*",
        async_mode="threading",
    )
    active_scans: dict[str, ScanRequest] = {}
    motion_dispatch_lock = threading.Lock()
    last_state_payload: Dict[str, object] | None = None
    last_state_emit_at = 0.0

    def start_motion_task(target, *args) -> bool:
        if not motion_dispatch_lock.acquire(blocking=False):
            return False

        def runner():
            try:
                target(*args)
            except Exception:
                log.exception("motion task")
            finally:
                motion_dispatch_lock.release()

        threading.Thread(target=runner, daemon=True).start()
        return True

    def serial_payload() -> Dict[str, object]:
        status = motion.serial_status()
        return {
            **status,
            "ports": discover_serial_ports(str(status.get("port") or "")),
        }

    # --- streaming wiring ---
    def push_state(state):
        nonlocal last_state_payload, last_state_emit_at
        payload = state.to_dict()
        payload["scan_running"] = scan.running
        now = time.monotonic()
        if payload == last_state_payload and (now - last_state_emit_at) < STATE_HEARTBEAT_S:
            return
        socketio.emit("state", payload)
        last_state_payload = dict(payload)
        last_state_emit_at = now

    def push_motion_state_if_available():
        state = getattr(motion, "state", None)
        if state is None or not hasattr(state, "to_dict"):
            return
        push_state(state)

    motion.subscribe(push_state)

    def push_serial(entry: Dict[str, object]):
        serial_console.append(entry)
        socketio.emit("serial_entry", entry)

    if hasattr(motion, "subscribe_serial"):
        motion.subscribe_serial(push_serial)

    def on_started(req: ScanRequest):
        active_scans[req.scan_id] = req
        push_motion_state_if_available()
        socketio.emit("scan_started", req.to_dict())

    def on_point(pt):
        req = active_scans.get(pt.scan_id)
        if req is not None:
            scan_history.append_point(req, pt)
        socketio.emit("scan_point", pt.to_dict())

    def on_complete(scan_id: str):
        active_scans.pop(scan_id, None)
        push_motion_state_if_available()
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
        if scan.running or motion.is_busy() or not start_motion_task(motion.home_all):
            return jsonify({"ok": False, "error": "controller busy"}), 409
        return jsonify({"ok": True})

    @app.post("/api/jog")
    def post_jog():
        data: Dict[str, Any] = request.get_json(force=True) or {}
        try:
            if not motion.serial_status()["connected"]:
                return jsonify({"ok": False, "error": "serial not connected"}), 409
            if scan.running or motion.is_busy():
                return jsonify({"ok": False, "error": "controller busy"}), 409
            if "x" in data and "y" in data:
                x, y = float(data["x"]), float(data["y"])
                motion.bounds.check(x, y)
                if not start_motion_task(motion.move_to, x, y):
                    return jsonify({"ok": False, "error": "controller busy"}), 409
            else:
                dx, dy = float(data.get("dx", 0)), float(data.get("dy", 0))
                current_x = motion.state.x
                current_y = motion.state.y
                if motion.state.homed and not motion.bounds.contains(current_x, current_y):
                    return jsonify(
                        {
                            "ok": False,
                            "error": (
                                f"current position ({current_x:.3f}, {current_y:.3f}) is outside workspace "
                                f"[{motion.bounds.x_min}, {motion.bounds.x_max}] x "
                                f"[{motion.bounds.y_min}, {motion.bounds.y_max}]; "
                                "home the machine or widen machine.bounds in config.yaml"
                            ),
                        }
                    ), 400
                # Pre-check against the last-polled position so obvious
                # out-of-range jogs return a clean 400 instead of failing
                # silently in the worker thread. The state snapshot is up to
                # ~1 s stale (1 Hz polling) — borderline jogs still
                # re-check against Marlin's fresh position inside motion.jog.
                if motion.state.homed:
                    motion.bounds.check(current_x + dx, current_y + dy)
                if not start_motion_task(motion.jog, dx, dy):
                    return jsonify({"ok": False, "error": "controller busy"}), 409
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

    @app.post("/api/block/override")
    def post_block_override():
        return jsonify({"ok": True, "overridden": motion.override_block()})

    @app.post("/api/fault/clear")
    def post_fault_clear():
        return jsonify({"ok": True, "cleared": motion.clear_fault()})

    @app.post("/api/scan")
    def post_scan():
        data = request.get_json(force=True) or {}
        if not motion.serial_status()["connected"]:
            return jsonify({"ok": False, "error": "serial not connected"}), 409
        try:
            req = ScanRequest(
                x_max=float(data["x_max"]),
                probe_target_x=float(data.get("probe_target_x", DEFAULT_PROBE_TARGET_X)),
                probe_speed_mm_s=float(data.get("probe_speed_mm_s", motion.scan_feed_mm_s)),
                y_max=float(data["y_max"]),
                y_min=float(data["y_min"]),
                n_samples=int(data["n_samples"]),
                scan_id=uuid.uuid4().hex[:8],
            )
        except (KeyError, ValueError) as e:
            return jsonify({"ok": False, "error": f"bad request: {e}"}), 400

        for x in (req.probe_target_x, req.x_max):
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
