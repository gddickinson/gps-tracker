"""Web interface tests: every exported endpoint answers, the static build is complete, and the JS engine tests pass.

Run: python -m pytest -q tests/test_web.py
"""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from web.app import STATIC, STATIC_ENDPOINTS, app  # noqa: E402

client = TestClient(app)


def test_index_served():
    r = client.get("/")
    assert r.status_code == 200 and "static/main.js" in r.text


@pytest.mark.parametrize("path", STATIC_ENDPOINTS)
def test_endpoint(path):
    r = client.get("/api" + path)
    assert r.status_code == 200, r.text
    assert r.json() is not None


def test_static_build(tmp_path):
    sys.path.insert(0, str(ROOT / "scripts"))
    import build_static
    build_static.build(tmp_path / "site")
    assert (tmp_path / "site" / "index.html").read_text().count("APP_STATIC") == 1
    for path in STATIC_ENDPOINTS:
        assert (tmp_path / "site" / "api" / (path.strip("/") + ".json")).exists()


def test_static_assets_referenced_exist():
    html = (STATIC / "index.html").read_text()
    for name in ("style.css", "main.js"):
        assert f"static/{name}" in html and (STATIC / name).exists()


@pytest.mark.skipif(not shutil.which("node"), reason="node not installed")
def test_js_engine():
    tests = sorted(STATIC.glob("*.test.mjs"))
    for t in tests:
        r = subprocess.run(["node", str(t)], capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr


FIX = ROOT / "tests" / "fixtures"


def test_info_lists_formats_and_settings():
    info = client.get("/api/info").json()
    assert info["formats"] == ["gpx", "tcx", "csv", "nmea"]
    assert info["settings"]["moving_speed_mps"] == 0.5 and "OpenStreetMap" in info["tile_servers"]


def test_demo_tracks_are_the_synthetic_fixtures():
    demos = client.get("/api/demo").json()
    assert len(demos) == 4
    for d in demos:
        assert d["text"] == (FIX / d["name"]).read_text(encoding="utf-8")


def test_analyze_runs_python_modules():
    import track_io
    import track_stats
    text = (FIX / "demo_route.gpx").read_text(encoding="utf-8")
    r = client.post("/api/analyze", files={"file": ("demo_route.gpx", text.encode())})
    assert r.status_code == 200, r.text
    expected = track_stats.compute_stats(track_io.parse_track(text, "x.gpx")["points"])
    assert r.json()["stats"]["distance_m"] == pytest.approx(expected["distance_m"])
    r = client.post("/api/analyze?split_m=1609.344&moving_speed=2", files={"file": ("demo_route.gpx", text.encode())})
    assert r.json()["stats"]["settings"]["split_m"] == 1609.344


def test_analyze_rejects_bad_files_and_settings():
    r = client.post("/api/analyze", files={"file": ("x.gpx", b"<gpx><trk>")})
    assert r.status_code == 400 and "XML" in r.json()["detail"]
    r = client.post("/api/analyze?split_m=0", files={"file": ("x.csv", b"lat,lon\n1,2\n")})
    assert r.status_code == 400


def test_static_site_has_no_tile_cache_or_real_tracks(tmp_path):
    sys.path.insert(0, str(ROOT / "scripts"))
    import build_static
    build_static.build(tmp_path / "site")
    names = [p.name for p in (tmp_path / "site").rglob("*")]
    assert not any(n.endswith((".db", ".csv", ".gpx", ".tcx", ".nmea")) for n in names)
