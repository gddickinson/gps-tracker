#!/usr/bin/env python3
"""
Enhanced GPS Tracker with Multi-Source Location Support and Interactive Mapping

This script provides a complete GPS tracking solution with the following features:
- Multiple location data sources (Serial GPS, IP Geolocation)
- Real-time position tracking and visualization
- Interactive map with multiple layer options
- Local tile caching for offline use
- Track recording and visualization
- Data logging capabilities

"""

import sys
import serial
import time
from datetime import datetime
import math
import os
import sqlite3
import requests
import platform
import csv
from abc import ABC, abstractmethod
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                           QVBoxLayout, QHBoxLayout, QLabel, QGridLayout,
                           QPushButton, QFileDialog, QTabWidget, QDialog,
                           QComboBox, QProgressBar, QFrame, QMessageBox)
from PyQt6.QtCore import QTimer, Qt, QPointF, QRectF, pyqtSignal
from PyQt6.QtGui import (QPainter, QPen, QColor, QBrush, QPixmap,
                        QPainterPath, QFont)
import pynmea2

# ===================== Location Provider Classes =====================

class LocationProvider(ABC):
    """
    Abstract base class for location providers.
    Defines the interface that all location providers must implement.
    """

    @abstractmethod
    def get_location(self):
        """
        Get current location data.
        Returns: dict with standardized location data fields
        """
        pass

    @abstractmethod
    def close(self):
        """Clean up provider resources"""
        pass

    @property
    @abstractmethod
    def name(self):
        """Get provider name for UI display"""
        pass

class SerialGPSProvider(LocationProvider):
    """
    Handles GPS data from serial port devices.
    Supports NMEA protocol parsing and connection management.
    """
    def __init__(self, port='/dev/tty.usbmodem101', baudrate=115200):
        self._name = f"Serial GPS ({port})"
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.track_history = []
        self.connect()

    def connect(self):
        """Establish connection to serial port"""
        try:
            if not self.serial_port or not self.serial_port.is_open:
                self.serial_port = serial.Serial(self.port, self.baudrate)
                print(f"Connected to {self.port}")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            self.serial_port = None

    def get_location(self):
        """
        Read and parse GPS data from serial port.
        Returns: dict containing current GPS data
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.connect()
            if not self.serial_port:
                return None

        try:
            line = self.serial_port.readline().decode('ascii', errors='replace')
            data = {
                'latitude': None, 'longitude': None, 'altitude': None,
                'speed': None, 'satellites': None, 'fix_quality': None,
                'time': None, 'date': None, 'track_angle': None,
                'hdop': None, 'source': self.name
            }

            if line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                data['time'] = msg.timestamp
                data['date'] = msg.datestamp
                if msg.status == 'A':  # Valid fix
                    data['latitude'] = msg.latitude
                    data['longitude'] = msg.longitude
                    data['speed'] = msg.spd_over_grnd
                    data['track_angle'] = msg.true_course

                    # Store position history
                    if data['latitude'] and data['longitude']:
                        self.track_history.append((data['latitude'], data['longitude']))
                        if len(self.track_history) > 1000:  # Keep last 1000 points
                            self.track_history.pop(0)

            elif line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                data['fix_quality'] = msg.gps_qual
                data['satellites'] = msg.num_sats
                data['altitude'] = msg.altitude
                data['hdop'] = msg.horizontal_dil

            return data

        except (serial.SerialException, pynmea2.ParseError) as e:
            print(f"Error reading GPS: {e}")
            self.close()
            return None

    def close(self):
        """Close serial port connection"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print(f"Closed {self.port}")
            except:
                pass

    @property
    def name(self):
        return self._name

class IPLocationProvider(LocationProvider):
    """
    Provides location data based on IP address.
    Includes caching to minimize API requests.
    """
    def __init__(self):
        self._name = "IP Geolocation"
        self.track_history = []
        self.last_update = 0
        self.cache_duration = 60  # Update every 60 seconds
        self.cached_data = None

    def get_location(self):
        """
        Get location data from IP geolocation service.
        Uses caching to limit API requests.
        """
        current_time = time.time()

        # Return cached data if available and recent
        if self.cached_data and (current_time - self.last_update) < self.cache_duration:
            return self.cached_data

        try:
            # Use ip-api.com (free, no API key required)
            response = requests.get('http://ip-api.com/json/', timeout=5)
            if response.status_code == 200:
                result = response.json()
                data = {
                    'latitude': result.get('lat'),
                    'longitude': result.get('lon'),
                    'altitude': None,
                    'speed': None,
                    'satellites': None,
                    'fix_quality': None,
                    'time': datetime.now().time(),
                    'date': datetime.now().date(),
                    'track_angle': None,
                    'hdop': None,
                    'source': self.name
                }

                if data['latitude'] and data['longitude']:
                    self.track_history.append((data['latitude'], data['longitude']))
                    if len(self.track_history) > 1000:
                        self.track_history.pop(0)

                self.cached_data = data
                self.last_update = current_time
                return data
        except Exception as e:
            print(f"Error getting IP location: {e}")
        return None

    def close(self):
        """Cleanup resources (no-op for IP provider)"""
        pass

    @property
    def name(self):
        return self._name

# ===================== Map Components =====================

class TileManager:
    """Manages map tiles with local caching and rate limiting"""
    def __init__(self, cache_dir="map_cache"):
        self.cache_dir = cache_dir
        self.tile_servers = {
            'OpenStreetMap': {
                'url': 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                'backup_urls': [
                    'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png',
                    'https://b.tile.openstreetmap.org/{z}/{x}/{y}.png',
                    'https://c.tile.openstreetmap.org/{z}/{x}/{y}.png'
                ],
                'user_agent': 'GPSTracker/1.0'
            },
            'OpenTopoMap': {
                'url': 'https://tile.opentopomap.org/{z}/{x}/{y}.png',
                'user_agent': 'GPSTracker/1.0'
            },
            'CyclOSM': {
                'url': 'https://c.tile-cyclosm.openstreetmap.fr/cyclosm/{z}/{x}/{y}.png',
                'user_agent': 'GPSTracker/1.0'
            }
        }
        self.current_server = 'OpenStreetMap'
        self.download_timeout = 3  # seconds
        self.last_download_time = {}  # Track last download time per server
        self.min_download_interval = 0.1  # seconds between downloads
        self._init_cache()

    def _init_cache(self):
        """Initialize SQLite database for tile caching"""
        os.makedirs(self.cache_dir, exist_ok=True)
        self.db_path = os.path.join(self.cache_dir, 'tile_cache.db')

        with sqlite3.connect(self.db_path) as conn:
            conn.execute('''CREATE TABLE IF NOT EXISTS tiles
                          (key TEXT PRIMARY KEY, data BLOB, timestamp TEXT)''')

    def _check_internet(self):
        """Check if internet connection is available"""
        try:
            requests.get("http://www.google.com", timeout=1)
            return True
        except requests.RequestException:
            return False

    def _respect_rate_limit(self, server):
        """Ensure we don't exceed rate limits"""
        if server in self.last_download_time:
            elapsed = time.time() - self.last_download_time[server]
            if elapsed < self.min_download_interval:
                time.sleep(self.min_download_interval - elapsed)
        self.last_download_time[server] = time.time()

    def get_tile(self, x, y, zoom):
        """Get a map tile, either from cache or download"""
        tile_key = f"{self.current_server}/{zoom}/{x}/{y}"

        # Check cache first
        try:
            with sqlite3.connect(self.db_path) as conn:
                result = conn.execute('SELECT data FROM tiles WHERE key = ?',
                                    (tile_key,)).fetchone()
                if result:
                    pixmap = QPixmap()
                    pixmap.loadFromData(result[0])
                    if not pixmap.isNull():
                        return pixmap
        except sqlite3.Error as e:
            print(f"Cache error: {e}")

        # If not in cache, try to download
        server_info = self.tile_servers[self.current_server]
        headers = {'User-Agent': server_info['user_agent']}

        # Try main URL first
        self._respect_rate_limit(self.current_server)
        success = self._try_download_tile(server_info['url'], x, y, zoom, headers, tile_key)
        if success:
            return success

        # If main URL fails and we have backup URLs, try them
        if 'backup_urls' in server_info:
            for backup_url in server_info['backup_urls']:
                self._respect_rate_limit(self.current_server)
                success = self._try_download_tile(backup_url, x, y, zoom, headers, tile_key)
                if success:
                    return success

        # Return default tile if all downloads fail
        return self._get_default_tile(x, y, zoom)

    def _try_download_tile(self, url_template, x, y, zoom, headers, tile_key):
        """Attempt to download a tile from a specific URL"""
        url = url_template.format(x=x, y=y, z=zoom)
        try:
            response = requests.get(url, timeout=self.download_timeout, headers=headers)
            if response.status_code == 200:
                # Cache the tile
                try:
                    with sqlite3.connect(self.db_path) as conn:
                        conn.execute('INSERT OR REPLACE INTO tiles VALUES (?, ?, ?)',
                                   (tile_key, response.content,
                                    datetime.now().isoformat()))
                except sqlite3.Error as e:
                    print(f"Cache write error: {e}")

                pixmap = QPixmap()
                if pixmap.loadFromData(response.content):
                    return pixmap
        except (requests.RequestException, requests.Timeout) as e:
            print(f"Download error for {url}: {e}")
        return None

    def _get_default_tile(self, x, y, zoom):
        """Create an informative default tile"""
        pixmap = QPixmap(256, 256)
        pixmap.fill(QColor(240, 240, 240))
        painter = QPainter(pixmap)

        # Draw grid
        painter.setPen(QPen(QColor(200, 200, 200)))
        painter.drawRect(0, 0, 255, 255)

        # Draw tile coordinates
        font = QFont()
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(10, 30, f"x: {x}")
        painter.drawText(10, 50, f"y: {y}")
        painter.drawText(10, 70, f"zoom: {zoom}")

        if not self._check_internet():
            painter.drawText(10, 120, "No internet connection")
        else:
            painter.drawText(10, 120, "Tile download failed")
            painter.drawText(10, 140, f"Server: {self.current_server}")

        painter.end()
        return pixmap

    def change_server(self, server_name):
        """Change the current tile server"""
        if server_name in self.tile_servers:
            self.current_server = server_name
            return True
        return False

class ScaleBar(QWidget):
    """
    Widget displaying map scale bar that updates with zoom level and latitude.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(200, 30)
        self.zoom_level = 15
        self.latitude = 0

    def update_scale(self, zoom, latitude):
        """Update scale bar for new zoom level and latitude"""
        self.zoom_level = zoom
        self.latitude = latitude
        self.update()

    def paintEvent(self, event):
        """Draw the scale bar"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Calculate scale
        meters_per_pixel = 156543.03392 * math.cos(math.radians(self.latitude)) / (2 ** self.zoom_level)
        scale_width = 100  # pixels
        distance = meters_per_pixel * scale_width

        # Round to a nice number
        if distance >= 1000:
            distance = round(distance / 1000) * 1000
            label = f"{distance/1000:.0f} km"
        else:
            distance = round(distance / 100) * 100
            label = f"{distance:.0f} m"

        # Draw scale bar
        painter.setPen(QPen(Qt.GlobalColor.black, 2))
        y = self.height() - 5
        painter.drawLine(10, y, 10 + scale_width, y)
        painter.drawLine(10, y-5, 10, y+5)
        painter.drawLine(10 + scale_width, y-5, 10 + scale_width, y+5)

        # Draw label
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)
        painter.drawText(QRectF(10, 0, scale_width, y-5),
                        Qt.AlignmentFlag.AlignCenter, label)

class MapWidget(QWidget):
    """
    Interactive map widget with layers, controls, and track visualization.
    Supports panning, zooming, and multiple map types.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 400)

        # Initialize map state
        self.zoom_level = 15
        self.center_lat = 0
        self.center_lon = 0
        self.tracks = []
        self.dragging = False
        self.last_pos = None

        # Initialize components
        self.tile_manager = TileManager()

        # Setup UI
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Status message
        self.status_message = QLabel()
        self.status_message.setStyleSheet("color: red;")
        layout.addWidget(self.status_message)

        # Controls
        controls = QHBoxLayout()

        # Map type selector
        self.map_type = QComboBox()
        self.map_type.addItems(self.tile_manager.tile_servers.keys())
        self.map_type.currentTextChanged.connect(self.change_map_type)
        controls.addWidget(QLabel("Map Type:"))
        controls.addWidget(self.map_type)

        # Force refresh button
        refresh_button = QPushButton("Refresh Map")
        refresh_button.clicked.connect(self.force_refresh)
        controls.addWidget(refresh_button)

        # Zoom controls
        zoom_in = QPushButton("+")
        zoom_out = QPushButton("-")
        zoom_in.clicked.connect(self.zoom_in)
        zoom_out.clicked.connect(self.zoom_out)
        controls.addWidget(zoom_in)
        controls.addWidget(zoom_out)

        # Scale bar
        self.scale_bar = ScaleBar()
        controls.addWidget(self.scale_bar)

        controls.addStretch()
        layout.addLayout(controls)

        # Status bar
        self.status_bar = QLabel()
        layout.addWidget(self.status_bar)

        # Check internet connection
        self.check_connection()

    def check_connection(self):
        """Check internet connection and update status"""
        if not self.tile_manager._check_internet():
            self.status_message.setText("Warning: No internet connection. Using cached tiles if available.")
        else:
            self.status_message.setText("")

    def force_refresh(self):
        """Force map refresh and connection check"""
        self.check_connection()
        self.update()

    def set_center(self, lat, lon):
        """Set the map center coordinates"""
        self.center_lat = lat
        self.center_lon = lon
        self.scale_bar.update_scale(self.zoom_level, lat)
        self.update()

    def update_tracks(self, tracks):
        """Update track data and redraw"""
        self.tracks = tracks
        self.update()

    def latlon_to_pixel(self, lat, lon):
        """Convert latitude/longitude to pixel coordinates"""
        n = 2.0 ** self.zoom_level
        lat_rad = math.radians(lat)
        x = ((lon + 180.0) / 360.0 * n)
        y = ((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return x, y

    def pixel_to_latlon(self, x, y):
        """Convert pixel coordinates to latitude/longitude"""
        n = 2.0 ** self.zoom_level
        lon_deg = x / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
        lat_deg = math.degrees(lat_rad)
        return lat_deg, lon_deg

    def paintEvent(self, event):
        """Draw the map and overlays"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Calculate tile coordinates
        center_x, center_y = self.latlon_to_pixel(self.center_lat, self.center_lon)
        tile_x = int(center_x)
        tile_y = int(center_y)

        # Calculate pixel offsets
        pixel_x = int((center_x - tile_x) * 256)
        pixel_y = int((center_y - tile_y) * 256)

        # Draw visible tiles
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                x = tile_x + dx
                y = tile_y + dy

                if 0 <= x < 2**self.zoom_level and 0 <= y < 2**self.zoom_level:
                    tile = self.tile_manager.get_tile(x, y, self.zoom_level)
                    # Convert coordinates to integers and create QRect for drawing
                    draw_x = int(self.width()/2 + (dx*256) - pixel_x)
                    draw_y = int(self.height()/2 + (dy*256) - pixel_y)
                    painter.drawPixmap(draw_x, draw_y, tile)

        # Draw tracks
        if self.tracks:
            painter.setPen(QPen(QColor(255, 0, 0, 180), 3))
            path = QPainterPath()
            first = True

            for lat, lon in self.tracks:
                x, y = self.latlon_to_pixel(lat, lon)
                screen_x = int(self.width()/2 + (x - center_x) * 256)
                screen_y = int(self.height()/2 + (y - center_y) * 256)

                if first:
                    path.moveTo(screen_x, screen_y)
                    first = False
                else:
                    path.lineTo(screen_x, screen_y)

            painter.drawPath(path)

            # Draw current position
            last_lat, last_lon = self.tracks[-1]
            x, y = self.latlon_to_pixel(last_lat, last_lon)
            screen_x = int(self.width()/2 + (x - center_x) * 256)
            screen_y = int(self.height()/2 + (y - center_y) * 256)

            painter.setBrush(QBrush(QColor(255, 0, 0)))
            painter.drawEllipse(QPointF(screen_x, screen_y), 5, 5)

        # Update status bar
        self.status_bar.setText(
            f"Center: {self.center_lat:.6f}, {self.center_lon:.6f} | "
            f"Zoom: {self.zoom_level}"
        )

    def zoom_in(self):
        """Increase zoom level"""
        self.zoom_level = min(19, self.zoom_level + 1)
        self.scale_bar.update_scale(self.zoom_level, self.center_lat)
        self.update()

    def zoom_out(self):
        """Decrease zoom level"""
        self.zoom_level = max(1, self.zoom_level - 1)
        self.scale_bar.update_scale(self.zoom_level, self.center_lat)
        self.update()

    def change_map_type(self, map_type):
        """Switch between different map tile sources"""
        if self.tile_manager.change_server(map_type):
            self.update()

    def mousePressEvent(self, event):
        """Handle mouse press for map dragging"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = True
            self.last_pos = event.pos()

    def mouseReleaseEvent(self, event):
        """Handle mouse release for map dragging"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = False

    def mouseMoveEvent(self, event):
        """Handle mouse movement for map dragging"""
        if self.dragging and self.last_pos:
            dx = event.pos().x() - self.last_pos.x()
            dy = event.pos().y() - self.last_pos.y()

            # Convert pixel movement to lat/lon
            n = 2.0 ** self.zoom_level
            dlat = -dy / 256.0 / n * 360.0
            dlon = -dx / 256.0 / n * 360.0

            self.center_lat += dlat
            self.center_lon += dlon
            self.last_pos = event.pos()
            self.update()

    def wheelEvent(self, event):
        """Handle mouse wheel for zooming"""
        delta = event.angleDelta().y()
        if delta > 0:
            self.zoom_in()
        elif delta < 0:
            self.zoom_out()

class MapDialog(QDialog):
    """Dialog window containing the map widget"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("GPS Track View")
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        self.map_widget = MapWidget()
        layout.addWidget(self.map_widget)

        self.setLayout(layout)

    def update_position(self, track_points):
        """Update the displayed track and center position"""
        if track_points:
            # Update track on map
            self.map_widget.update_tracks(track_points)

            # Center on current position
            current_lat, current_lon = track_points[-1]
            self.map_widget.set_center(current_lat, current_lon)

class LocationProviderManager:
    """
    Manages multiple location data providers.
    Handles provider detection, switching, and lifecycle.
    """
    def __init__(self):
        self.providers = {}
        self.active_provider = None
        self.detect_available_providers()

    def detect_available_providers(self):
        """Detect and initialize available location providers"""
        # Always add IP location as it's always available
        self.providers['ip'] = IPLocationProvider()

        # Check for serial GPS devices
        self.detect_serial_devices()

    def detect_serial_devices(self):
        """Detect available serial GPS devices"""
        if platform.system() == 'Darwin':  # macOS
            self._detect_macos_devices()
        elif platform.system() == 'Linux':
            self._detect_linux_devices()
        elif platform.system() == 'Windows':
            self._detect_windows_devices()

    def _detect_macos_devices(self):
        """Detect GPS devices on macOS"""
        import glob
        patterns = [
            '/dev/tty.usbmodem*',  # USB modems
            '/dev/tty.usbserial*', # USB serial devices
            '/dev/tty.SLAB_*',     # Silicon Labs devices
            '/dev/tty.GPS*'        # Explicitly named GPS devices
        ]

        for pattern in patterns:
            for port in glob.glob(pattern):
                try:
                    # Try to open the port briefly to check if it's available
                    test_serial = serial.Serial(port, 115200, timeout=1)
                    test_serial.close()

                    # Try to read some data and check if it looks like GPS
                    with serial.Serial(port, 115200, timeout=1) as ser:
                        for _ in range(5):  # Try reading a few lines
                            line = ser.readline().decode('ascii', errors='replace').strip()
                            if line.startswith('$GP'):  # NMEA sentence detected
                                provider = SerialGPSProvider(port)
                                self.providers[f'serial_{port}'] = provider
                                print(f"Found GPS device on {port}")
                                break

                except (serial.SerialException, OSError):
                    continue

    def _detect_linux_devices(self):
        """Detect GPS devices on Linux"""
        import glob
        patterns = [
            '/dev/ttyUSB*',    # USB to Serial adapters
            '/dev/ttyACM*',    # USB modems
            '/dev/ttyS*',      # Hardware serial ports
            '/dev/gps*'        # Explicitly named GPS devices
        ]

        for pattern in patterns:
            for port in glob.glob(pattern):
                try:
                    with serial.Serial(port, 115200, timeout=1) as ser:
                        for _ in range(5):
                            line = ser.readline().decode('ascii', errors='replace').strip()
                            if line.startswith('$GP'):
                                provider = SerialGPSProvider(port)
                                self.providers[f'serial_{port}'] = provider
                                print(f"Found GPS device on {port}")
                                break
                except:
                    continue

    def _detect_windows_devices(self):
        """Detect GPS devices on Windows"""
        import serial.tools.list_ports

        ports = serial.tools.list_ports.comports()

        for port in ports:
            try:
                if any(keyword in port.description.lower()
                      for keyword in ['gps', 'location', 'navigation']):
                    provider = SerialGPSProvider(port.device)
                    self.providers[f'serial_{port.device}'] = provider
                    print(f"Found GPS device on {port.device}")
                    continue

                # If not obviously a GPS, try reading from it
                with serial.Serial(port.device, 115200, timeout=1) as ser:
                    for _ in range(5):
                        line = ser.readline().decode('ascii', errors='replace').strip()
                        if line.startswith('$GP'):
                            provider = SerialGPSProvider(port.device)
                            self.providers[f'serial_{port.device}'] = provider
                            print(f"Found GPS device on {port.device}")
                            break
            except:
                continue

    def refresh_devices(self):
        """Refresh the list of available devices"""
        # Close all existing serial providers
        for provider in self.providers.values():
            if isinstance(provider, SerialGPSProvider):
                provider.close()

        # Remove all serial providers
        self.providers = {k:v for k,v in self.providers.items()
                         if not isinstance(v, SerialGPSProvider)}

        # Detect devices again
        self.detect_serial_devices()

    def get_provider_names(self):
        """Get list of available provider names"""
        return [provider.name for provider in self.providers.values()]

    def set_active_provider(self, provider_name):
        """Set the active location provider"""
        for provider in self.providers.values():
            if provider.name == provider_name:
                if self.active_provider:
                    self.active_provider.close()
                self.active_provider = provider
                return True
        return False

    def get_location(self):
        """Get location data from active provider"""
        if self.active_provider:
            return self.active_provider.get_location()
        return None

    def get_track_history(self):
        """Get track history from active provider"""
        if self.active_provider:
            return self.active_provider.track_history
        return []

    def close(self):
        """Clean up all providers"""
        for provider in self.providers.values():
            provider.close()


class GPSWindow(QMainWindow):
    """
    Main application window for GPS tracker.
    Manages UI, data collection, and visualization.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Enhanced GPS Tracker")
        self.setGeometry(100, 100, 600, 400)

        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()
        main_widget.setLayout(layout)

        # Create source selection
        source_layout = QHBoxLayout()
        source_label = QLabel("Location Source:")
        self.source_combo = QComboBox()
        self.location_manager = LocationProviderManager()

        # Populate source combo box
        self.source_combo.addItems(self.location_manager.get_provider_names())
        self.source_combo.currentTextChanged.connect(self.change_location_source)

        # Add refresh button
        refresh_button = QPushButton("Refresh Devices")
        refresh_button.clicked.connect(self.refresh_devices)

        source_layout.addWidget(source_label)
        source_layout.addWidget(self.source_combo)
        source_layout.addWidget(refresh_button)
        layout.addLayout(source_layout)

        # Create tab widget
        tabs = QTabWidget()
        layout.addWidget(tabs)

        # Basic info tab
        basic_tab = QWidget()
        basic_layout = QVBoxLayout()
        basic_tab.setLayout(basic_layout)

        # Create labels for GPS data
        self.labels = {}
        grid_layout = QGridLayout()

        label_names = [
            'Time', 'Date', 'Latitude', 'Longitude',
            'Speed', 'Track Angle', 'Fix Quality', 'Satellites',
            'Altitude', 'HDOP', 'Source', 'Distance'
        ]

        for i, name in enumerate(label_names):
            label = QLabel(f"{name}:")
            value = QLabel("N/A")
            value.setStyleSheet("font-weight: bold;")
            grid_layout.addWidget(label, i % 6, (i // 6) * 2)
            grid_layout.addWidget(value, i % 6, (i // 6) * 2 + 1)
            self.labels[name.lower()] = value

        basic_layout.addLayout(grid_layout)

        # Add control buttons
        button_layout = QHBoxLayout()

        self.log_button = QPushButton("Start Logging")
        self.log_button.clicked.connect(self.toggle_logging)
        button_layout.addWidget(self.log_button)

        map_button = QPushButton("Show Map")
        map_button.clicked.connect(self.show_map)
        button_layout.addWidget(map_button)

        clear_track_button = QPushButton("Clear Track")
        clear_track_button.clicked.connect(self.clear_track)
        button_layout.addWidget(clear_track_button)

        basic_layout.addLayout(button_layout)
        tabs.addTab(basic_tab, "Basic Info")

        # Initialize map dialog and logging state
        self.map_dialog = None
        self.logging = False
        self.log_file = None
        self.writer = None

        # Initialize with first available provider
        if self.source_combo.count() > 0:
            self.location_manager.set_active_provider(self.source_combo.currentText())

        # Start update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gps)
        self.timer.start(1000)  # Update every second

    def show_map(self):
        """Show the map dialog"""
        if not self.map_dialog:
            self.map_dialog = MapDialog(self)
        self.map_dialog.show()
        self.map_dialog.raise_()

    def refresh_devices(self):
        """Refresh available location sources"""
        current_source = self.source_combo.currentText()
        self.location_manager.refresh_devices()

        # Update combo box
        self.source_combo.clear()
        self.source_combo.addItems(self.location_manager.get_provider_names())

        # Try to restore previous selection
        index = self.source_combo.findText(current_source)
        if index >= 0:
            self.source_combo.setCurrentIndex(index)

        QMessageBox.information(self, "Devices Refreshed",
                              "Available location sources have been refreshed.")

    def change_location_source(self, source_name):
        """Change the active location source"""
        if self.location_manager.set_active_provider(source_name):
            self.clear_track()
            QMessageBox.information(self, "Source Changed",
                                  f"Successfully switched to {source_name}")
        else:
            QMessageBox.warning(self, "Error",
                              f"Failed to switch to {source_name}")

    def toggle_logging(self):
        """Start or stop data logging"""
        if not self.logging:
            filename, _ = QFileDialog.getSaveFileName(self, "Save GPS Log", "", "CSV Files (*.csv)")
            if filename:
                self.log_file = open(filename, 'w', newline='')
                fieldnames = ['timestamp', 'latitude', 'longitude', 'speed',
                            'altitude', 'satellites', 'source']
                self.writer = csv.DictWriter(self.log_file, fieldnames=fieldnames)
                self.writer.writeheader()
                self.logging = True
                self.log_button.setText("Stop Logging")
        else:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.writer = None
            self.logging = False
            self.log_button.setText("Start Logging")

    def update_gps(self):
        """Update GPS data display"""
        data = self.location_manager.get_location()
        if data:
            # Update basic info
            if data.get('time'):
                self.labels['time'].setText(data['time'].strftime('%H:%M:%S'))
            if data.get('date'):
                self.labels['date'].setText(data['date'].strftime('%Y-%m-%d'))
            if data.get('latitude'):
                self.labels['latitude'].setText(f"{data['latitude']:.6f}")
            if data.get('longitude'):
                self.labels['longitude'].setText(f"{data['longitude']:.6f}")
            if data.get('speed'):
                try:
                    speed_val = float(data['speed'])
                    self.labels['speed'].setText(f"{speed_val:.1f} knots")
                except (ValueError, TypeError):
                    self.labels['speed'].setText(str(data['speed']))
            if data.get('track_angle'):
                try:
                    angle_val = float(data['track_angle'])
                    self.labels['track angle'].setText(f"{angle_val:.1f}°")
                except (ValueError, TypeError):
                    self.labels['track angle'].setText(str(data['track_angle']))
            if data.get('fix_quality') is not None:
                self.labels['fix quality'].setText(str(data['fix_quality']))
            if data.get('satellites'):
                self.labels['satellites'].setText(str(data['satellites']))
            if data.get('altitude'):
                try:
                    alt_val = float(data['altitude'])
                    self.labels['altitude'].setText(f"{alt_val:.1f}m")
                except (ValueError, TypeError):
                    self.labels['altitude'].setText(str(data['altitude']))
            if data.get('hdop'):
                try:
                    hdop_val = float(data['hdop'])
                    self.labels['hdop'].setText(f"{hdop_val:.1f}")
                except (ValueError, TypeError):
                    self.labels['hdop'].setText(str(data['hdop']))
            if data.get('source'):
                self.labels['source'].setText(str(data['source']))

            # Update map if visible
            if self.map_dialog and self.map_dialog.isVisible():
                self.map_dialog.update_position(self.location_manager.get_track_history())

            # Log data if enabled
            if self.logging and self.writer and data.get('latitude'):
                self.writer.writerow({
                    'timestamp': datetime.now().isoformat(),
                    'latitude': data.get('latitude'),
                    'longitude': data.get('longitude'),
                    'speed': data.get('speed'),
                    'altitude': data.get('altitude'),
                    'satellites': data.get('satellites'),
                    'source': data.get('source')
                })

    def clear_track(self):
        """Clear the current track history"""
        if self.location_manager.active_provider:
            self.location_manager.active_provider.track_history.clear()
        if self.map_dialog:
            self.map_dialog.update_position([])

    def closeEvent(self, event):
        """Handle application shutdown"""
        self.location_manager.close()
        if self.map_dialog:
            self.map_dialog.close()
        if self.log_file:
            self.log_file.close()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = GPSWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
