from src.core.scan import ScanPoint, ScanRequest
from src.web.plot_viewer import create_plot_viewer_app
from src.web.scan_history import ScanHistoryCsvStore


def test_plot_viewer_root_serves_plot_page(tmp_path):
    csv_path = tmp_path / "scan_history.csv"
    app = create_plot_viewer_app(scan_history_path=csv_path)

    with app.test_client() as client:
        response = client.get("/")
        body = response.get_data(as_text=True)
        assert response.status_code == 200
        assert "Wood FPM Plot Viewer" in body
        assert "plot_viewer.js" in body
        assert "id=\"plot\"" in body


def test_plot_viewer_csv_and_version_follow_history_updates(tmp_path):
    csv_path = tmp_path / "scan_history.csv"
    store = ScanHistoryCsvStore(csv_path)
    app = create_plot_viewer_app(scan_history_store=store)

    req = ScanRequest(
        x_max=8.0,
        probe_target_x=-2.0,
        y_max=5.0,
        y_min=-5.0,
        n_samples=3,
        probe_speed_mm_s=3.5,
        scan_id="viewer123",
    )
    pt = ScanPoint(scan_id="viewer123", index=1, x=2.5, y=1.0)

    with app.test_client() as client:
        initial_version = client.get("/api/scan/history/version").get_json()
        store.append_point(req, pt)

        updated_version = client.get("/api/scan/history/version").get_json()
        body = client.get("/api/scan/history.csv").get_data(as_text=True)

        assert updated_version["size"] > initial_version["size"]
        assert updated_version["mtime_ns"] >= initial_version["mtime_ns"]
        assert "viewer123" in body
        assert "scan_id,index,hit,x,y" in body