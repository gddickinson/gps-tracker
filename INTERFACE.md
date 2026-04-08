# GPS Tracker -- Interface Map

## Module Overview

| File | Purpose |
|------|---------|
| `main.py` | Entry point -- sets up logging, launches PyQt6 application |
| `providers.py` | Location data sources -- abstract base, serial GPS, IP geolocation, provider manager |
| `mapping.py` | Map display -- tile management/caching, scale bar, interactive map widget, map dialog |
| `gui.py` | Main window -- GPSWindow with data display, logging controls, source switching |
| `gps-tracker.py` | Original monolithic file (kept for reference) |
| `test_providers.py` | Smoke tests for providers and manager |

## Key Classes and Functions

### providers.py
- `LocationProvider(ABC)` -- abstract interface for location sources
- `SerialGPSProvider` -- reads NMEA data from serial GPS devices
- `IPLocationProvider` -- IP-based geolocation with caching (ip-api.com)
- `LocationProviderManager` -- discovers, manages, and switches between providers

### mapping.py
- `TileManager` -- downloads and caches map tiles in SQLite; supports multiple tile servers
- `ScaleBar(QWidget)` -- dynamic scale bar that updates with zoom/latitude
- `MapWidget(QWidget)` -- interactive map with pan, zoom, track overlay
- `MapDialog(QDialog)` -- wrapper dialog for MapWidget

### gui.py
- `GPSWindow(QMainWindow)` -- main application window
  - GPS data label grid, source selector, logging controls, map button
  - `update_gps()` -- timer-driven data refresh
  - `toggle_logging()` -- start/stop CSV logging

### main.py
- `main()` -- configures logging, creates QApplication, shows GPSWindow

## Module Dependencies
```
main.py --> gui.py --> providers.py
                   --> mapping.py
```

## Configuration
- Tile server URLs are defined in `mapping.TILE_SERVERS` (no hardcoded paths)
- Tile cache stored in `map_cache/tile_cache.db` (SQLite)
