import threading
from pathlib import Path

import src.web.app as web_app

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


def test_serial_api_connect_disconnect_cycle(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
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


def test_discover_serial_ports_includes_virtual_pts(monkeypatch):
    def fake_glob(self, pattern):
        assert self == Path("/dev")
        matches = {
            "ttyACM*": [Path("/dev/ttyACM0")],
            "ttyUSB*": [Path("/dev/ttyUSB0")],
            "pts/[0-9]*": [Path("/dev/pts/7"), Path("/dev/pts/2")],
        }
        return matches.get(pattern, [])

    monkeypatch.setattr(web_app.Path, "glob", fake_glob)

    assert web_app.discover_serial_ports() == [
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
        "/dev/pts/2",
        "/dev/pts/7",
    ]


def test_duplicate_state_emits_are_throttled(tmp_path, monkeypatch):
    motion = build_motion()
    try:
        _app, socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        emitted = []
        monotonic_values = iter([0.0, 0.2, 1.3])

        monkeypatch.setattr(web_app.time, "monotonic", lambda: next(monotonic_values))
        monkeypatch.setattr(socketio, "emit", lambda event, payload: emitted.append((event, payload)))

        motion._broadcast()  # noqa: SLF001
        motion._broadcast()  # noqa: SLF001
        motion._broadcast()  # noqa: SLF001

        assert [event for event, _payload in emitted] == ["state", "state"]
        assert emitted[0][1] == emitted[1][1]
    finally:
        motion.shutdown()


def test_serial_api_can_switch_out_of_simulation(tmp_path, monkeypatch):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
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


def test_fault_can_be_cleared(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        motion._target = (5.0, 0.0)  # noqa: SLF001
        motion._busy_flag = True  # noqa: SLF001
        motion._last_counts = (1, 2, 3)  # noqa: SLF001
        motion._stable_count_polls = 2  # noqa: SLF001
        motion._set_state(busy=True, fault=True, last_error="timeout waiting for move to complete")  # noqa: SLF001

        with app.test_client() as client:
            cleared = client.post("/api/fault/clear")
            payload = cleared.get_json()
            assert cleared.status_code == 200
            assert payload["ok"] is True
            assert payload["cleared"] is True
            assert motion.state.busy is False
            assert motion.state.fault is False
            assert motion.state.last_error == ""
            assert motion._target is None  # noqa: SLF001
            assert motion._busy_flag is False  # noqa: SLF001
            assert motion._last_counts is None  # noqa: SLF001
            assert motion._stable_count_polls == 0  # noqa: SLF001

            cleared_again = client.post("/api/fault/clear")
            assert cleared_again.get_json()["cleared"] is False
    finally:
        motion.shutdown()


def test_busy_without_fault_can_be_cleared(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        motion._target = {motion._x_letter: 5.0, motion._y_letter: 0.0}  # noqa: SLF001
        motion._busy_flag = True  # noqa: SLF001
        motion._last_counts = {motion._x_letter: 100}  # noqa: SLF001
        motion._stable_count_polls = 1  # noqa: SLF001
        motion._set_state(busy=True, fault=False, last_error="")  # noqa: SLF001

        with app.test_client() as client:
            cleared = client.post("/api/fault/clear")
            payload = cleared.get_json()
            assert cleared.status_code == 200
            assert payload["ok"] is True
            assert payload["cleared"] is True
            assert motion.state.busy is False
            assert motion.state.fault is False
            assert motion.state.last_error == ""
            assert motion._target is None  # noqa: SLF001
            assert motion._busy_flag is False  # noqa: SLF001
            assert motion._last_counts is None  # noqa: SLF001
            assert motion._stable_count_polls == 0  # noqa: SLF001
    finally:
        motion.shutdown()


def test_override_block_releases_busy_without_clearing_fault(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        motion._target = {motion._x_letter: 5.0, motion._y_letter: 0.0}  # noqa: SLF001
        motion._busy_flag = True  # noqa: SLF001
        motion._last_counts = {motion._x_letter: 100}  # noqa: SLF001
        motion._stable_count_polls = 1  # noqa: SLF001
        motion._set_state(busy=True, fault=True, last_error="timeout waiting for move to complete")  # noqa: SLF001

        with app.test_client() as client:
            overridden = client.post("/api/block/override")
            payload = overridden.get_json()
            assert overridden.status_code == 200
            assert payload["ok"] is True
            assert payload["overridden"] is True
            assert motion.state.busy is False
            assert motion.state.fault is True
            assert motion.state.last_error == "timeout waiting for move to complete"
            assert motion._target is None  # noqa: SLF001
            assert motion._busy_flag is False  # noqa: SLF001
            assert motion._last_counts is None  # noqa: SLF001
            assert motion._stable_count_polls == 0  # noqa: SLF001

            overridden_again = client.post("/api/block/override")
            assert overridden_again.get_json()["overridden"] is False
    finally:
        motion.shutdown()


def test_jog_reports_current_position_outside_workspace(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        motion._set_state(x=12.0, y=0.0, homed=True)  # noqa: SLF001

        with app.test_client() as client:
            jog = client.post("/api/jog", json={"dx": -1.0, "dy": 0.0})
            payload = jog.get_json()
            assert jog.status_code == 400
            assert "current position (12.000, 0.000) is outside workspace" in payload["error"]
            assert "config.yaml" in payload["error"]
    finally:
        motion.shutdown()


def test_unhomed_jog_skips_absolute_workspace_precheck(tmp_path, monkeypatch):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        calls = []

        class ImmediateThread:
            def __init__(self, target=None, args=(), daemon=None, kwargs=None):
                self._target = target
                self._args = args
                self._kwargs = kwargs or {}

            def start(self):
                if self._target is not None:
                    self._target(*self._args, **self._kwargs)

        monkeypatch.setattr(web_app.threading, "Thread", ImmediateThread)
        monkeypatch.setattr(motion, "jog", lambda dx, dy: calls.append((dx, dy)))
        motion._set_state(x=12.0, y=0.0, homed=False)  # noqa: SLF001

        with app.test_client() as client:
            jog = client.post("/api/jog", json={"dx": -1.0, "dy": 0.0})
            payload = jog.get_json()
            assert jog.status_code == 200
            assert payload["ok"] is True
            assert calls == [(-1.0, 0.0)]
    finally:
        motion.shutdown()


def test_jog_rejects_when_controller_busy(tmp_path):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        motion._set_state(busy=True)  # noqa: SLF001

        with app.test_client() as client:
            jog = client.post("/api/jog", json={"dx": 1.0, "dy": 0.0})
            assert jog.status_code == 409
            assert jog.get_json()["error"] == "controller busy"
    finally:
        motion.shutdown()


def test_jog_rejects_when_previous_jog_is_dispatching(tmp_path, monkeypatch):
    motion = build_motion()
    try:
        app, _socketio = create_app(
            motion=motion,
            scan=FakeScan(),
            scan_history_path=tmp_path / "scan_history.csv",
        )

        started = threading.Event()
        release = threading.Event()

        def blocking_jog(dx, dy):
            started.set()
            assert (dx, dy) == (1.0, 0.0)
            release.wait(timeout=2.0)

        monkeypatch.setattr(motion, "jog", blocking_jog)
        motion._set_state(homed=False)  # noqa: SLF001

        with app.test_client() as client:
            first = client.post("/api/jog", json={"dx": 1.0, "dy": 0.0})
            assert first.status_code == 200
            assert started.wait(timeout=1.0)

            second = client.post("/api/jog", json={"dx": 1.0, "dy": 0.0})
            assert second.status_code == 409
            assert second.get_json()["error"] == "controller busy"

            release.set()
    finally:
        motion.shutdown()