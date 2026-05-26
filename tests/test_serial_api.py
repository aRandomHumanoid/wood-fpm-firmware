import src.web.app as web_app

from src.core.limits import MachineBounds, MotionLimits
from src.core.motion import MotionController
from src.web.app import create_app


class FakeProbe:
    def on_change(self, _fn):
        pass


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


def test_serial_api_connect_disconnect_cycle(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            probe=FakeProbe(),
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        with app.test_client() as client:
            status = client.get("/api/serial").get_json()
            assert status["connected"] is True
            assert status["port"] == "/dev/ttyACM0"
            assert status["baudrate"] == 250000

            disconnected = client.post("/api/serial/disconnect").get_json()
            assert disconnected["ok"] is True
            assert disconnected["connected"] is False

            reconnected = client.post(
                "/api/serial/connect",
                json={"port": "/dev/ttyUSB0", "baudrate": 115200},
            ).get_json()
            assert reconnected["ok"] is True
            assert reconnected["connected"] is True
            assert reconnected["port"] == "/dev/ttyUSB0"
            assert reconnected["baudrate"] == 115200

            disconnected_again = client.post("/api/serial/disconnect").get_json()
            assert disconnected_again["connected"] is False

            blocked_home = client.post("/api/home")
            assert blocked_home.status_code == 409
            assert blocked_home.get_json()["error"] == "serial not connected"
    finally:
        motion.shutdown()


def test_serial_api_blocks_toggle_while_busy(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            probe=FakeProbe(),
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        motion._set_state(busy=True)  # noqa: SLF001
        with app.test_client() as client:
            disconnect = client.post("/api/serial/disconnect")
            assert disconnect.status_code == 409
            assert disconnect.get_json()["error"] == "controller busy"

            connect = client.post("/api/serial/connect", json={"port": "/dev/ttyUSB0", "baudrate": 115200})
            assert connect.status_code == 409
            assert connect.get_json()["error"] == "controller busy"
    finally:
        motion.shutdown()


def test_serial_status_includes_discovered_ports(tmp_path, monkeypatch):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            probe=FakeProbe(),
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        monkeypatch.setattr(
            web_app,
            "discover_serial_ports",
            lambda preferred_port=None: ["/dev/ttyUSB0", "/dev/ttyACM0", preferred_port],
        )

        with app.test_client() as client:
            status = client.get("/api/serial").get_json()
            assert status["ports"][0] == "/dev/ttyUSB0"
            assert "/dev/ttyACM0" in status["ports"]
            assert status["port"] in status["ports"]
    finally:
        motion.shutdown()


def test_serial_api_can_switch_out_of_simulation(tmp_path, monkeypatch):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            probe=FakeProbe(),
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        original_connect_driver = motion._connect_driver  # noqa: SLF001

        def fake_connect_driver(marlin_port, marlin_baudrate, simulate):
            if simulate:
                return original_connect_driver(marlin_port, marlin_baudrate, simulate)
            motion._simulate = False  # noqa: SLF001
            motion._marlin_port = marlin_port  # noqa: SLF001
            motion._marlin_baudrate = marlin_baudrate  # noqa: SLF001
            motion._set_state(serial_connected=True, fault=False, last_error="")  # noqa: SLF001

        monkeypatch.setattr(motion, "_connect_driver", fake_connect_driver)

        with app.test_client() as client:
            disconnected = client.post("/api/serial/disconnect")
            assert disconnected.status_code == 200

            reconnected = client.post(
                "/api/serial/connect",
                json={"port": "/dev/ttyUSB0", "baudrate": 115200, "simulate": False},
            )
            payload = reconnected.get_json()
            assert reconnected.status_code == 200
            assert payload["ok"] is True
            assert payload["simulate"] is False
            assert payload["port"] == "/dev/ttyUSB0"
            assert payload["baudrate"] == 115200
    finally:
        motion.shutdown()


def test_estop_can_be_reset(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            probe=FakeProbe(),
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        with app.test_client() as client:
            stopped = client.post("/api/stop")
            assert stopped.status_code == 200
            assert motion.state.last_error == "E-STOP"
            assert motion.state.fault is True

            reset = client.post("/api/stop/reset")
            payload = reset.get_json()
            assert reset.status_code == 200
            assert payload["ok"] is True
            assert payload["reset"] is True
            assert motion.state.last_error == ""
            assert motion.state.fault is False

            reset_again = client.post("/api/stop/reset")
            assert reset_again.get_json()["reset"] is False
    finally:
        motion.shutdown()