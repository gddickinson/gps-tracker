# GPS Tracker -- Interface Map

## Module Overview

| File | Purpose |
|------|---------|
| `main.py` | Entry point -- sets up logging, launches PyQt6 application |
| `providers.py` | Location data sources -- abstract base, serial GPS (NMEA RMC+GGA), IP geolocation, provider manager |
| `mapping.py` | Map display -- tile management/caching, scale bar, interactive map widget, map dialog |
| `gui.py` | Main window -- GPSWindow with data display, track distance, logging controls, source switching |
| `geo.py` | Qt-free maths: Web Mercator tile projection, map panning, scale bar distance, haversine distances |
| `tile_servers.py` | `TILE_SERVERS` -- tile URL configuration (Qt-free; used by mapping.py and the web app) |
| `track_values.py` | Strict number / ISO time parsing and the track point dict shared by the readers |
| `track_io.py` | Read recorded tracks: GPX, TCX, CSV (incl. this app's log); `parse_track()` dispatches by format |
| `track_nmea.py` | Read raw NMEA logs with pynmea2 (RMC points, GGA altitude) |
| `track_stats.py` | `compute_stats()` -- distance, duration, moving time, speeds, elevation gain/loss, heart rate, splits |
| `gps-tracker.py` | Original monolithic file (kept for reference) |
| `test_providers.py` | Smoke tests for providers and manager |
| `run_web.py` | Starts the web app (uvicorn) on http://127.0.0.1:8161 |
| `web/app.py` | FastAPI app: `/api/info`, `/api/demo`, `POST /api/analyze`; `STATIC_ENDPOINTS` for the static export |
| `web/static/` | Front end (ES modules, no build). Pages: `home.js`, `track.js`, `about.js`; `track_map.js` (SVG map), `track_charts.js` (elevation profile, splits), `format.js`; JS ports: `geo.js`, `track_values.js`, `xml_lite.js`, `track_io.js`, `track_nmea.js`, `track_stats.js`; `core.js`/`style.css` shared web kit; `static_api.js` answers api() on the static site |
| `web/static/track_engine.test.mjs` | Node test: JS ports equal the Python values in `tests/fixtures/track_expected.json` |
| `scripts/build_static.py` | Export the static GitHub Pages site to `site/` |
| `scripts/screenshots.py` | Playwright smoke test / README screenshots (`--static` or `--base URL`) |
| `scripts/make_demo_tracks.py` | Write the synthetic demo tracks (GPX, TCX, CSV log, NMEA) to `tests/fixtures/` |
| `scripts/make_track_fixture.py` | Write `tests/fixtures/track_expected.json` (Python results for the JS test) |
| `tests/` | pytest: geo, track readers/stats, JS fixture current, JS-vs-Python fuzz (`test_js_port.py` + `js_port_runner.mjs`), web API/static build, desktop map/provider/GUI regressions (Qt offscreen, fake serial) |

## Key Classes and Functions

### providers.py
- `LocationProvider(ABC)` -- abstract interface for location sources
- `SerialGPSProvider` -- reads every waiting NMEA sentence (any talker) and merges RMC + GGA into one state
- `IPLocationProvider` -- IP-based geolocation with caching (ip-api.com)
- `LocationProviderManager` -- discovers, manages, and switches between providers

### mapping.py
- `TileManager` -- downloads and caches map tiles in SQLite; supports multiple tile servers
- `ScaleBar(QWidget)` -- scale bar of a round distance, drawn at its true length (`geo.nice_scale`)
- `MapWidget(QWidget)` -- interactive map with pan (`geo.pan`), zoom, track overlay
- `MapDialog(QDialog)` -- wrapper dialog for MapWidget

### gui.py
- `GPSWindow(QMainWindow)` -- main application window
  - `update_gps()` -- timer-driven data refresh; `_update_distance()` -- track length (`geo.path_length_m`)
  - `toggle_logging()` -- start/stop CSV logging

### geo.py
- `latlon_to_tile`, `tile_to_latlon`, `pan`, `meters_per_pixel`, `nice_scale`, `haversine_m`, `path_length_m`

### track_io.py / track_nmea.py / track_stats.py
- `parse_track(text, filename)` -> `{format, name, points, skipped}`; points are `{lat, lon, ele, time, hr, seg}`
- `parse_nmea(text)`; `compute_stats(points, moving_speed, speed_window, elevation_threshold, split_m)`

### main.py
- `main()` -- configures logging, creates QApplication, shows GPSWindow

## Module Dependencies
```
main.py --> gui.py --> providers.py
                   --> mapping.py --> geo.py, tile_servers.py
                   --> geo.py
web/app.py --> track_io.py --> track_values.py, track_nmea.py
           --> track_stats.py --> geo.py
           --> tile_servers.py
web/static/track.js --> track_io.js, track_stats.js, track_map.js, track_charts.js (JS ports of the above)
```

## Configuration
- Tile server URLs are defined in `tile_servers.TILE_SERVERS` (no hardcoded paths)
- Tile cache stored in `map_cache/tile_cache.db` (SQLite)
- Statistic defaults: `track_stats.MOVING_SPEED_MPS`, `SPEED_WINDOW_S`, `ELEVATION_THRESHOLD_M`, `SPLIT_M`
