"""FastAPI web app for the GPS tracker: serves the front end in static/ and the JSON API under /api.

Run with `python run_web.py` (http://127.0.0.1:8161, local only). `scripts/build_static.py` exports every path in
STATIC_ENDPOINTS to JSON for the GitHub Pages site, so keep that list in step with the GET routes below. Track files
are read and analysed in the browser (static/track_io.js, track_stats.js: ports of track_io.py, track_stats.py);
the local app also offers POST /api/analyze, which runs the Python modules on an uploaded file for comparison.
"""
import sys
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import track_io  # noqa: E402
import track_stats  # noqa: E402
from tile_servers import TILE_SERVERS  # noqa: E402

STATIC = Path(__file__).resolve().parent / "static"
FIXTURES = ROOT / "tests" / "fixtures"
DEMOS = {  # synthetic tracks written by scripts/make_demo_tracks.py (invented route at 0 N, 0 E)
    "demo_route.gpx": "GPX: the synthetic loop, two segments, elevation and heart rate",
    "demo_log.csv": "CSV in this app's own log format (what 'Start Logging' writes)",
    "demo_log.nmea": "NMEA: raw RMC + GGA sentences from a receiver (first 10 minutes)",
    "demo_route.tcx": "TCX: the same loop as a Garmin Training Center file",
}
MAX_UPLOAD_BYTES = 50 * 1024 * 1024

app = FastAPI(title="GPS Tracker")
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/", include_in_schema=False)
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/info")
def info():
    """Project summary, the statistic settings (from track_stats) and the desktop app's map sources."""
    return {
        "title": "GPS Tracker",
        "description": ("A Python GPS tracking application with real-time position tracking, data visualisation "
                        "and interactive mapping, with serial (NMEA) GPS and IP geolocation sources."),
        "formats": list(track_io.FORMATS),
        "csv_columns": {k: list(v) for k, v in track_io.CSV_COLUMNS.items()},
        "settings": {
            "moving_speed_mps": track_stats.MOVING_SPEED_MPS,
            "speed_window_s": track_stats.SPEED_WINDOW_S,
            "elevation_threshold_m": track_stats.ELEVATION_THRESHOLD_M,
            "split_m": track_stats.SPLIT_M,
        },
        "tile_servers": list(TILE_SERVERS),
        "demos": len(DEMOS),
    }


@app.get("/api/demo")
def demo():
    """The synthetic demo tracks (text) for the "try the demo" buttons."""
    return [{"name": n, "label": label, "text": (FIXTURES / n).read_text(encoding="utf-8")}
            for n, label in DEMOS.items()]


@app.post("/api/analyze")
async def analyze(file: UploadFile = File(...), moving_speed: float = track_stats.MOVING_SPEED_MPS,
                  speed_window: float = track_stats.SPEED_WINDOW_S,
                  elevation_threshold: float = track_stats.ELEVATION_THRESHOLD_M, split_m: float = track_stats.SPLIT_M):
    """Parse an uploaded track and compute its statistics with the Python modules; nothing is stored."""
    data = await file.read()
    if len(data) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, "File too large for the web app (50 MB).")
    if min(speed_window, elevation_threshold, split_m) <= 0 or moving_speed < 0:
        raise HTTPException(400, "Settings must be positive.")
    try:
        parsed = track_io.parse_track(data.decode("utf-8", errors="replace"), Path(file.filename or "").name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    stats = track_stats.compute_stats(parsed["points"], moving_speed, speed_window, elevation_threshold, split_m)
    return {"parsed": parsed, "stats": stats}


# GET paths (without the /api prefix) written to the static site as api/<path>.json.
STATIC_ENDPOINTS = ["/info", "/demo"]
