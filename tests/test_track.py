"""track_io / track_nmea / track_stats: hand-checked values and the JS fixture being current."""
import json
import math
import sys
from pathlib import Path

import pytest

import geo
import track_io
import track_stats
from track_values import parse_time

ROOT = Path(__file__).resolve().parent.parent
FIX = ROOT / "tests" / "fixtures"


def pts(rows, seg=0):
    return [{"lat": la, "lon": lo, "ele": e, "time": t, "hr": None, "seg": seg} for la, lo, e, t in rows]


def test_distance_skips_segment_gaps():
    a = pts([(0, 0, None, 0), (0, 0.01, None, 10)], 0) + pts([(1, 1, None, 20), (1, 1.01, None, 30)], 1)
    s = track_stats.compute_stats(a)
    expected = geo.haversine_m(0, 0, 0, 0.01) + geo.haversine_m(1, 1, 1, 1.01)
    assert s["distance_m"] == pytest.approx(expected) and s["n_segments"] == 2


def test_moving_time_excludes_stops_and_max_speed_window():
    d = 10 / geo.EARTH_RADIUS_M * 180 / math.pi   # 10 m of longitude on the equator, in degrees
    rows = [(0, 0, None, 0), (0, d, None, 5), (0, d, None, 65), (0, 2 * d, None, 70), (0, 4 * d, None, 75)]
    s = track_stats.compute_stats(pts(rows))
    assert s["duration_s"] == 75 and s["moving_time_s"] == 15
    assert s["moving_distance_m"] == pytest.approx(40)
    assert s["max_speed_mps"] == pytest.approx(30 / 10)      # 65 s -> 75 s: 30 m in 10 s
    assert s["avg_speed_mps"] == pytest.approx(40 / 75)


def test_elevation_hysteresis():
    eles = [100, 101, 100.5, 103, 102, 98, 99, 98.5, 105]
    s = track_stats.compute_stats(pts([(0, i * 1e-4, e, None) for i, e in enumerate(eles)]))
    assert s["ele_gain_m"] == pytest.approx(3 + 7) and s["ele_loss_m"] == pytest.approx(5)
    assert s["duration_s"] is None and s["moving_time_s"] is None and s["splits"] == []


def test_splits_interpolate_crossing_times():
    d = 600 / geo.EARTH_RADIUS_M * 180 / math.pi
    s = track_stats.compute_stats(pts([(0, 0, None, 0), (0, d, None, 120), (0, 2 * d, None, 360)]))
    assert [x["distance_m"] for x in s["splits"]] == pytest.approx([1000, 200])
    assert s["splits"][0]["time_s"] == pytest.approx(120 + 400 / 600 * 240)


def test_times():
    assert parse_time("1970-01-01T00:00:00Z") == 0
    assert parse_time("2024-05-01T10:00:00+02:00") == parse_time("2024-05-01T08:00:00Z")
    assert parse_time("2023-02-29T00:00:00Z") is None


def test_demo_formats_agree():
    """GPX and TCX of the demo route give identical statistics; the CSV log and NMEA give the same points."""
    s = {n: track_stats.compute_stats(track_io.parse_track((FIX / n).read_text(), n)["points"])
         for n in ("demo_route.gpx", "demo_route.tcx")}
    assert s["demo_route.gpx"]["distance_m"] == s["demo_route.tcx"]["distance_m"]
    csv_pts = track_io.parse_track((FIX / "demo_log.csv").read_text(), "demo_log.csv")["points"]
    gpx_pts = track_io.parse_track((FIX / "demo_route.gpx").read_text(), "demo_route.gpx")["points"]
    assert [(p["lat"], p["lon"], p["ele"]) for p in csv_pts] == [(p["lat"], p["lon"], p["ele"]) for p in gpx_pts]
    nmea = track_io.parse_track((FIX / "demo_log.nmea").read_text(), "demo_log.nmea")
    assert len(nmea["points"]) == 118 and nmea["skipped"] == 1     # one lost fix (ignored), one bad checksum
    assert nmea["points"][0]["lat"] == pytest.approx(gpx_pts[0]["lat"], abs=1e-6)


def test_errors_are_value_errors():
    for text, name in [("", "a.csv"), ("x,y\n1,2", "a.csv"), ("<gpx", "a.gpx"), ("<a/>", "a.tcx")]:
        with pytest.raises(ValueError):
            track_io.parse_track(text, name)


def _close(a, b):
    if isinstance(a, float) or isinstance(b, float):
        return a is not None and b is not None and math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(_close(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)):
        return isinstance(b, (list, tuple)) and len(a) == len(b) and all(_close(x, y) for x, y in zip(a, b))
    return a == b


def test_js_fixture_is_current():
    sys.path.insert(0, str(ROOT / "scripts"))
    import make_track_fixture
    fresh = json.loads(json.dumps(make_track_fixture.build()))
    saved = json.loads((FIX / "track_expected.json").read_text())
    assert _close(fresh, saved), "run python scripts/make_track_fixture.py"
