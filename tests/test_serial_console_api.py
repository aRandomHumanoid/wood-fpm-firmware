from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import MotionController
from src.web.app import create_app


class FakeScan:
    def __init__(self):
        self.running = False
        self.on_started = None
        self.on_point = None
        self.on_complete = None

    def abort(self):
        self.running = False


def build_motion():
    return MotionController(
        bounds=MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0),
        limits=MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0),
        marlin_port="/dev/ttyACM0",
        marlin_baudrate=250000,
        simulate=True,
    )


def test_serial_console_records_manual_and_ui_commands(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        with app.test_client() as client:
            manual = client.post("/api/serial/command", json={"command": "M115"})
            assert manual.status_code == 200
            assert manual.get_json()["ok"] is True

            home = client.post("/api/home")
            assert home.status_code == 200

            console = client.get("/api/serial/console").get_json()["entries"]
            lines = [entry["line"] for entry in console]
            directions = [entry["direction"] for entry in console]
            assert any(direction == "tx" and line == "M115" for direction, line in zip(directions, lines))
            assert any(direction == "tx" and line == "G28 X Y" for direction, line in zip(directions, lines))
            assert any(direction == "rx" and "FIRMWARE_NAME:Marlin-sim" in line for direction, line in zip(directions, lines))

            cleared = client.post("/api/serial/console/clear")
            assert cleared.status_code == 200
            assert client.get("/api/serial/console").get_json()["entries"] == []
    finally:
        motion.shutdown()