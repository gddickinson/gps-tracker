# Session log

## 2026-11-12 -- web interface, static site and bug fixes (Claude Code)

**Added**
- Qt-free modules: `geo.py` (projection, pan, scale bar, haversine), `tile_servers.py`, `track_values.py`,
  `track_io.py` (GPX/TCX/CSV), `track_nmea.py` (NMEA logs via pynmea2), `track_stats.py` (distance, moving time,
  speeds, elevation gain/loss with a 2 m hysteresis, heart rate, splits).
- Web interface (`web/`, `run_web.py`, port 8161) and static GitHub Pages site (`scripts/build_static.py`,
  `.github/workflows/pages.yml`): Home, Analyse a track (file drop, synthetic demos, settings, stats tiles, SVG route
  map with pan/zoom/scale bar and optional OpenStreetMap tiles, elevation profile, splits), About.
- JS ports of the track modules; node test `web/static/track_engine.test.mjs` (10,751 checks against
  `tests/fixtures/track_expected.json`), seeded fuzz test `tests/test_js_port.py` (240 mutated files per run; 10,000+
  more during development, 0 differences after fixing the port gaps it found: XML declaration/PI/comment/entity/
  namespace rules, CDATA outside the root, Python's whitespace set).
- Synthetic demo tracks (`scripts/make_demo_tracks.py`): invented loop at 0 N, 0 E. No real GPS data is published.

**Bugs fixed in the desktop app** (separate commits, regression tests in `tests/`)
1. Map dragging moved the map vertically against the mouse and used a non-Mercator degrees/pixel rate.
2. Scale bar drawn 100 px long regardless of its rounded label (up to ~50% wrong, "0 m" at high zoom).
3. Serial GPS: one NMEA line read per second (growing lag), GGA altitude/satellites never in the CSV log, a corrupt
   line closed the port, `$GN`/`$GL` receivers ignored, 0.0 deg positions dropped.
4. GUI ignored zero values (speed 0 kept showing the last speed) and the Distance field was never filled in.

**Privacy / public data check**
- No GPS track files are committed. `map_cache/tile_cache.db` (5 MB, committed) holds 460 OpenStreetMap/OpenTopoMap/
  CyclOSM tiles; the zoom 16-17 tiles cover a ~1 km area of south Austin, TX, so they reveal where the app was used
  (probably George's location or his IP-geolocation position). Not touched; not in the static site
  (`test_static_site_has_no_tile_cache_or_real_tracks`). Needs George's decision: remove from the repo / .gitignore?

**Verification**: `pytest tests test_providers.py` 41 passed; node test ok; screenshots `--static` and `--base` all ok,
phone width 0 px overflow; local "Check with the Python modules" identical for all four demo formats.
The Mac mini was not used.

## 2026-11-12 -- follow-up (lead's request)
- `map_cache/tile_cache.db` untracked (`git rm --cached`) and `map_cache/` added to .gitignore: its zoom 16-17 tiles
  showed where the app was used. The file is still in git history (commits b6eb18b, 156dff0); removing it from
  history would need a history rewrite (George's decision). The local copy is kept, and the app recreates it if missing.
- Legacy `gps-tracker.py` moved to `_archive/` (superseded by main.py + modules; nothing imported it; it still has the
  bugs fixed in the modules).
