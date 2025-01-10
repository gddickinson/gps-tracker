# GPS Tracker with Real-Time Mapping

A Python-based GPS tracking application that provides real-time position tracking, data visualization, and interactive mapping capabilities. The application supports multiple location data sources and features a robust offline-capable mapping system.

## Features

- **Multiple Location Sources**
  - Serial GPS device support (NMEA protocol)
  - IP-based geolocation
  - Automatic GPS device detection
  - Easy switching between data sources

- **Interactive Mapping**
  - Real-time position tracking
  - Multiple map providers (OpenStreetMap, OpenTopoMap, CyclOSM)
  - Local tile caching for offline use
  - Scale bar and zoom controls
  - Track visualization
  - Draggable map interface

- **Data Collection**
  - Real-time GPS data display
  - CSV data logging
  - Track history recording
  - Basic statistics (speed, altitude, etc.)

- **User Interface**
  - Clean, modern PyQt6-based interface
  - Tabbed organization
  - Status indicators
  - Configuration options

## Requirements

### Hardware
- Optional: Serial GPS device (NMEA compatible)
- Computer running Windows, macOS, or Linux

### Software
- Python 3.11 or later
- Required Python packages (see Installation)

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/gps-tracker.git
cd gps-tracker
```

2. Create and activate a virtual environment (recommended):
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install required packages:
```bash
pip install pyserial pynmea2 PyQt6 requests
```

## Usage

1. Start the application:
```bash
python gps-tracker.py
```

2. Select a location source from the dropdown menu:
   - If using a GPS device, it should be automatically detected
   - IP Geolocation is always available as a fallback

3. Main interface features:
   - **Show Map**: Opens the interactive map window
   - **Start Logging**: Begin recording GPS data to a CSV file
   - **Clear Track**: Erases the current track history
   - **Refresh Devices**: Scan for new GPS devices

4. Map controls:
   - Click and drag to pan
   - Mouse wheel or +/- buttons to zoom
   - Drop-down to select different map styles
   - Scale bar shows current map scale

## Technical Details

### Architecture

The application is built with a modular design consisting of several key components:

#### Location Providers
- Abstract base class `LocationProvider` defines the interface
- `SerialGPSProvider`: Handles GPS device communication
- `IPLocationProvider`: Provides IP-based location
- `LocationProviderManager`: Manages provider lifecycle

#### Mapping System
- `TileManager`: Handles map tile downloading and caching
- `MapWidget`: Renders map and handles user interaction
- `ScaleBar`: Provides dynamic scale information

#### Data Management
- SQLite database for map tile caching
- CSV logging for track data
- Real-time data processing

### File Structure

```
gps-tracker/
├── gps-tracker.py     # Main application file
├── map_cache/         # Cached map tiles
│   └── tile_cache.db
├── requirements.txt   # Python dependencies
└── README.md
```

### Key Classes

#### GPSWindow
Main application window handling:
- User interface
- Data updates
- Provider management
- Logging control

#### TileManager
Map tile management including:
- Multiple server support
- Rate limiting
- Caching system
- Error handling

#### MapWidget
Interactive map display with:
- Tile rendering
- Track visualization
- User input handling
- Dynamic updates

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### Development Setup

1. Fork the repository
2. Create a feature branch
3. Follow the installation instructions above
4. Make your changes
5. Submit a pull request

### Guidelines

- Follow PEP 8 style guide
- Add comments for complex functionality
- Update documentation as needed
- Add tests for new features

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- OpenStreetMap and contributors for map data
- PyQt6 team for the GUI framework
- Various map tile providers

## Troubleshooting

### Common Issues

1. **No GPS devices detected**
   - Check USB connections
   - Verify device permissions
   - Try refreshing devices

2. **Map tiles not loading**
   - Check internet connection
   - Verify firewall settings
   - Try different map providers

3. **Serial port errors**
   - Check device permissions
   - Verify correct port selection
   - Ensure proper baud rate

### Debug Mode

Add the `--debug` flag when running for additional logging:
```bash
python gps-tracker.py --debug
```

## Future Enhancements

Planned features for future releases:
- Multiple simultaneous tracks
- Elevation profiles
- Route planning
- Weather data integration
- Additional map sources
- Mobile device support
- Network GPS receiver support
- Advanced statistics and analysis

## Support

For support, please:
1. Check the troubleshooting guide
2. Search existing issues
3. Create a new issue with:
   - Detailed problem description
   - Error messages
   - System information