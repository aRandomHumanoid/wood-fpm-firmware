"""Flask + Socket.IO web layer.

Endpoints:
  GET  /             — single-page UI
  POST /api/home     — kick off homing on both axes
  POST /api/jog      — {dx, dy} or {x, y}
  POST /api/scan     — {x_min, x_max, y_max, y_min, n_samples}
  POST /api/scan/abort
  POST /api/stop     — E-stop
  GET  /api/limits   — read-only vel/accel/bounds

Socket.IO events out:
  state         (~20 Hz)  — current pose / probe / busy / fault
  scan_started, scan_point, scan_complete
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from typing import Any, Dict

from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO

from ..core.limits import BoundsError
from ..core.motion import MotionController
from ..core.probe import ProbeMonitor
from ..core.scan import ScanRequest, ScanRunner

log = logging.getLogger(__name__)


def create_app(
    motion: MotionController,
    probe: ProbeMonitor,
    scan: ScanRunner,
) -> tuple[Flask, SocketIO]:
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
    )
    app.config["motion"] = motion
    app.config["probe"] = probe
    app.config["scan"] = scan

    socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

    # --- streaming wiring ---
    def push_state(state):
        socketio.emit("state", state.to_dict())

    motion.subscribe(push_state)

    def on_probe(triggered: bool):
        motion.set_probe_state(triggered)

    probe.on_change(on_probe)

    def on_started(req: ScanRequest):
        socketio.emit("scan_started", req.to_dict())

    def on_point(pt):
        socketio.emit("scan_point", pt.to_dict())

    def on_complete(scan_id: str):
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

    @app.post("/api/home")
    def post_home():
        threading.Thread(target=motion.home_all, daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/jog")
    def post_jog():
        data: Dict[str, Any] = request.get_json(force=True) or {}
        try:
            if "x" in data and "y" in data:
                threading.Thread(
                    target=motion.move_to,
                    args=(float(data["x"]), float(data["y"])),
                    daemon=True,
                ).start()
            else:
                threading.Thread(
                    target=motion.jog,
                    args=(float(data.get("dx", 0)), float(data.get("dy", 0))),
                    daemon=True,
                ).start()
            return jsonify({"ok": True})
        except BoundsError as e:
            return jsonify({"ok": False, "error": str(e)}), 400

    @app.post("/api/stop")
    def post_stop():
        scan.abort()
        motion.stop()
        return jsonify({"ok": True})

    @app.post("/api/scan")
    def post_scan():
        data = request.get_json(force=True) or {}
        try:
            req = ScanRequest(
                x_min=float(data["x_min"]),
                x_max=float(data["x_max"]),
                y_max=float(data["y_max"]),
                y_min=float(data["y_min"]),
                n_samples=int(data["n_samples"]),
                scan_id=uuid.uuid4().hex[:8],
            )
        except (KeyError, ValueError) as e:
            return jsonify({"ok": False, "error": f"bad request: {e}"}), 400

        for x in (req.x_min, req.x_max):
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

    return app, socketio
