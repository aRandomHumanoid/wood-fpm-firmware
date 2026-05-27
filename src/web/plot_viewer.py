"""Standalone plot-only viewer served on a dedicated port."""

from __future__ import annotations

from pathlib import Path

from flask import Flask, Response, jsonify, render_template

from .scan_history import ScanHistoryCsvStore


def create_plot_viewer_app(
    scan_history_path: Path | str = Path("scan_history.csv"),
    scan_history_store: ScanHistoryCsvStore | None = None,
) -> Flask:
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static",
    )
    scan_history = scan_history_store or ScanHistoryCsvStore(Path(scan_history_path))
    app.config["scan_history"] = scan_history

    @app.get("/")
    def index():
        return render_template("plot_viewer.html")

    @app.get("/api/scan/history.csv")
    def get_scan_history_csv():
        return Response(scan_history.read_text(), mimetype="text/csv")

    @app.get("/api/scan/history/version")
    def get_scan_history_version():
        stat = scan_history.path.stat()
        return jsonify({"mtime_ns": stat.st_mtime_ns, "size": stat.st_size})

    return app