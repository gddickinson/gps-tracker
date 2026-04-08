# GPS Tracker -- Roadmap

## Current State
A functional GPS tracking app with PyQt6 GUI, serial GPS and IP geolocation providers, interactive tile-based mapping, and CSV logging. The entire application lives in a single 1037-line file (`gps-tracker.py`), which makes maintenance difficult. No tests exist. The `GPS_I2C_Parsing` subfolder suggests I2C hardware support was started but is separate from the main app.

## Short-term Improvements
- [x] Split `gps-tracker.py` into modules: `providers.py` (LocationProvider, SerialGPSProvider, IPLocationProvider), `mapping.py` (TileManager, MapWidget, ScaleBar), `gui.py` (GPSWindow), and `main.py` (entry point) -- file is 1037 lines
- [ ] Add input validation for serial port selection and baud rate configuration
- [ ] Add error handling around tile download failures (network timeouts, corrupted tiles)
- [x] Create `requirements.txt` with pinned versions (currently lists packages but no versions)
- [ ] Add type hints throughout -- the codebase uses none
- [x] Add unit tests for LocationProvider subclasses and TileManager caching logic

## Feature Enhancements
- [ ] Integrate the `GPS_I2C_Parsing` module into the main app as an additional LocationProvider
- [ ] Add GPX file export/import for track data (industry standard format)
- [ ] Implement elevation profile visualization for recorded tracks
- [ ] Add distance and speed calculations with moving average smoothing
- [ ] Support multiple simultaneous tracks with color coding
- [ ] Add geofencing alerts (notify when entering/leaving an area)
- [ ] Implement tile pre-fetching for smoother map panning

## Long-term Vision
- [ ] Add route planning with waypoint support
- [ ] Implement a REST API mode so the tracker can serve location data to other apps
- [ ] Add weather overlay using OpenWeatherMap API
- [ ] Support MQTT for real-time fleet tracking across multiple devices
- [ ] Create a headless mode for Raspberry Pi deployment without GUI
- [ ] Package as a standalone app with PyInstaller

## Technical Debt
- [x] The monolithic `gps-tracker.py` violates separation of concerns -- split immediately
- [ ] SQLite tile cache (`map_cache/tile_cache.db`) has no eviction policy -- will grow unbounded
- [x] No logging framework -- uses bare print statements; switch to Python `logging`
- [x] Hard-coded tile server URLs should move to a configuration file
- [x] The `test_log.csv` is committed to the repo -- add to `.gitignore`
- [x] No `.gitignore` file exists
