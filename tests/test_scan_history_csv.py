from pathlib import Path

from src.core.limits import MachineBounds, MotionLimits
from src.core.scan import ScanPoint, ScanRequest
from src.web.app import create_app


class FakeMotion:
    def __init__(self):
        self.bounds = MachineBounds(x_min=0.0, x_max=10.0, y_min=-10.0, y_max=10.0)
        self.limits = MotionLimits(v_max_mm_s=20.0, a_max_mm_s2=100.0)
        self.rapid_feed_mm_s = 15.0
        self.scan_feed_mm_s = 2.0

    def subscribe(self, _fn):
        pass

    def set_probe_state(self, _triggered):
        pass


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


def test_scan_history_csv_routes_and_append(tmp_path):
    csv_path = tmp_path / "scan_history.csv"
    app, _socketio = create_app(
        motion=FakeMotion(),
        probe=FakeProbe(),
        scan=FakeScan(),
        scan_history_path=csv_path,
    )
    scan = app.config["scan"]

    req = ScanRequest(x_max=8.0, y_max=5.0, y_min=-5.0, n_samples=3, scan_id="scan1234")
    pt = ScanPoint(scan_id="scan1234", index=1, x=2.5, y=1.0)

    scan.on_started(req)
    scan.on_point(pt)
    scan.on_complete(req.scan_id)

    with app.test_client() as client:
        response = client.get("/api/scan/history.csv")
        body = response.get_data(as_text=True)
        assert response.status_code == 200
        assert response.mimetype == "text/csv"
        assert "scan_id,index,hit,x,y" in body
        assert "scan1234,1,True,2.500000,1.000000,8.000000,0.000000,-5.000000,5.000000,3" in body

        clear = client.post("/api/scan/history/clear")
        cleared = client.get("/api/scan/history.csv")
        assert clear.status_code == 200
        assert cleared.get_data(as_text=True).strip() == (
            "recorded_at_utc,scan_id,index,hit,x,y,x_max,probe_target_x,y_min,y_max,n_samples"
        )


def test_scan_history_clear_rejects_during_scan(tmp_path):
    csv_path = tmp_path / "scan_history.csv"
    app, _socketio = create_app(
        motion=FakeMotion(),
        probe=FakeProbe(),
        scan=FakeScan(),
        scan_history_path=csv_path,
    )
    app.config["scan"].running = True

    with app.test_client() as client:
        response = client.post("/api/scan/history/clear")
        assert response.status_code == 409
        assert response.get_json()["ok"] is False