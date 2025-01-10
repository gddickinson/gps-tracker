import sys
import serial
import time
from datetime import datetime
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                           QVBoxLayout, QHBoxLayout, QLabel, QGridLayout,
                           QPushButton, QFileDialog, QTabWidget, QDialog,
                           QComboBox, QMessageBox)
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush
import pynmea2
import csv
from math import radians, sin, cos, sqrt, atan2
import requests
import socket
import platform
import subprocess
import json
from abc import ABC, abstractmethod

class LocationProvider(ABC):
    """Abstract base class for location providers"""

    @abstractmethod
    def get_location(self):
        """Return location data in standardized format"""
        pass

    @abstractmethod
    def close(self):
        """Clean up resources"""
        pass

    @property
    @abstractmethod
    def name(self):
        """Provider name for UI"""
        pass

class SerialGPSProvider(LocationProvider):
    def __init__(self, port='/dev/tty.usbmodem101', baudrate=115200):
        self.serial_port = serial.Serial(port, baudrate)
        self._name = f"Serial GPS ({port})"
        self.track_history = []

    def get_location(self):
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
                if msg.status == 'A':
                    data['latitude'] = msg.latitude
                    data['longitude'] = msg.longitude
                    data['speed'] = msg.spd_over_grnd
                    data['track_angle'] = msg.true_course

                    if data['latitude'] and data['longitude']:
                        self.track_history.append((data['latitude'], data['longitude']))
                        if len(self.track_history) > 1000:
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
            return None

    def close(self):
        self.serial_port.close()

    @property
    def name(self):
        return self._name

class IPLocationProvider(LocationProvider):
    def __init__(self):
        self._name = "IP Geolocation"
        self.track_history = []
        self.last_update = 0
        self.cache_duration = 60  # Update every 60 seconds
        self.cached_data = None

    def get_location(self):
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
        pass

    @property
    def name(self):
        return self._name

class MobileDeviceProvider(LocationProvider):
    def __init__(self):
        self._name = "Mobile Device"
        self.track_history = []

    def get_location(self):
        # This is a placeholder - implementation would depend on:
        # 1. If running on mobile device directly
        # 2. If connecting to mobile device via USB/Bluetooth
        # 3. Platform-specific APIs
        return None

    def close(self):
        pass

    @property
    def name(self):
        return self._name

class LocationProviderManager:
    def __init__(self):
        self.providers = {}
        self.active_provider = None
        self.detect_available_providers()

    def detect_available_providers(self):
        # Always add IP location as it's always available
        self.providers['ip'] = IPLocationProvider()

        # Check for serial GPS devices
        self.detect_serial_devices()

        # Check for mobile device capabilities
        if self.check_mobile_capability():
            self.providers['mobile'] = MobileDeviceProvider()

    def detect_serial_devices(self):
        # Basic serial port detection
        if platform.system() == 'Darwin':  # macOS
            import glob
            ports = glob.glob('/dev/tty.*')
            for port in ports:
                if 'usbmodem' in port or 'uart' in port:
                    try:
                        provider = SerialGPSProvider(port)
                        self.providers[f'serial_{port}'] = provider
                    except:
                        pass
        # Add similar detection for Windows/Linux

    def check_mobile_capability(self):
        # Check if running on mobile device or has mobile device connected
        return False  # Placeholder

    def get_provider_names(self):
        return [provider.name for provider in self.providers.values()]

    def set_active_provider(self, provider_name):
        for provider in self.providers.values():
            if provider.name == provider_name:
                if self.active_provider:
                    self.active_provider.close()
                self.active_provider = provider
                return True
        return False

    def get_location(self):
        if self.active_provider:
            return self.active_provider.get_location()
        return None

    def get_track_history(self):
        if self.active_provider:
            return self.active_provider.track_history
        return []

    def close(self):
        for provider in self.providers.values():
            provider.close()



# class GPSReader:
#     def __init__(self, port='/dev/tty.usbmodem1101', baudrate=115200):
#         self.serial_port = serial.Serial(port, baudrate)
#         self.current_data = {
#             'latitude': None,
#             'longitude': None,
#             'time': None,
#             'date': None,
#             'speed': None,
#             'fix_quality': None,
#             'satellites': None,
#             'altitude': None,
#             'track_angle': None,
#             'hdop': None,
#             'distance_traveled': 0.0,
#             'max_speed': 0.0,
#             'last_position': None
#         }
#         self.logging = False
#         self.log_file = None
#         self.writer = None
#         self.track_history = []  # Store position history for plotting

#     def haversine_distance(self, lat1, lon1, lat2, lon2):
#         R = 6371  # Earth's radius in kilometers
#         dlat = radians(lat2 - lat1)
#         dlon = radians(lon2 - lon1)
#         a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
#         c = 2 * atan2(sqrt(a), sqrt(1-a))
#         return R * c

#     def read_gps(self):
#         try:
#             line = self.serial_port.readline().decode('ascii', errors='replace')

#             if line.startswith('$GPRMC'):
#                 msg = pynmea2.parse(line)
#                 self.current_data['time'] = msg.timestamp
#                 self.current_data['date'] = msg.datestamp
#                 if msg.status == 'A':  # Valid fix
#                     self.current_data['latitude'] = msg.latitude
#                     self.current_data['longitude'] = msg.longitude
#                     self.current_data['speed'] = msg.spd_over_grnd
#                     self.current_data['track_angle'] = msg.true_course

#                     # Store position history
#                     if msg.latitude and msg.longitude:
#                         self.track_history.append((msg.latitude, msg.longitude))
#                         # Keep last 1000 points
#                         if len(self.track_history) > 1000:
#                             self.track_history.pop(0)

#                     # Update distance traveled
#                     if self.current_data['last_position']:
#                         last_lat, last_lon = self.current_data['last_position']
#                         distance = self.haversine_distance(
#                             last_lat, last_lon,
#                             msg.latitude, msg.longitude
#                         )
#                         self.current_data['distance_traveled'] += distance

#                     self.current_data['last_position'] = (msg.latitude, msg.longitude)

#                     # Update max speed
#                     if msg.spd_over_grnd and msg.spd_over_grnd > self.current_data['max_speed']:
#                         self.current_data['max_speed'] = msg.spd_over_grnd

#             elif line.startswith('$GPGGA'):
#                 msg = pynmea2.parse(line)
#                 self.current_data['fix_quality'] = msg.gps_qual
#                 self.current_data['satellites'] = msg.num_sats
#                 self.current_data['altitude'] = msg.altitude
#                 self.current_data['hdop'] = msg.horizontal_dil

#             if self.logging and self.writer and self.current_data['latitude']:
#                 self.writer.writerow({
#                     'timestamp': datetime.now().isoformat(),
#                     'latitude': self.current_data['latitude'],
#                     'longitude': self.current_data['longitude'],
#                     'speed': self.current_data['speed'],
#                     'altitude': self.current_data['altitude'],
#                     'satellites': self.current_data['satellites']
#                 })

#             return self.current_data

#         except (serial.SerialException, pynmea2.ParseError) as e:
#             print(f"Error reading GPS: {e}")
#             return None

#     def start_logging(self, filename):
#         self.log_file = open(filename, 'w', newline='')
#         fieldnames = ['timestamp', 'latitude', 'longitude', 'speed', 'altitude', 'satellites']
#         self.writer = csv.DictWriter(self.log_file, fieldnames=fieldnames)
#         self.writer.writeheader()
#         self.logging = True

#     def stop_logging(self):
#         self.logging = False
#         if self.log_file:
#             self.log_file.close()
#             self.log_file = None
#             self.writer = None

#     def close(self):
#         self.stop_logging()
#         self.serial_port.close()

class TrackMapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.track_points = []
        self.center_lat = None
        self.center_lon = None
        self.zoom_level = 1.0
        self.setMinimumSize(400, 400)

        # Navigation controls
        layout = QVBoxLayout()
        nav_layout = QHBoxLayout()

        zoom_in = QPushButton("+")
        zoom_out = QPushButton("-")
        zoom_in.clicked.connect(self.zoom_in)
        zoom_out.clicked.connect(self.zoom_out)

        nav_layout.addWidget(zoom_in)
        nav_layout.addWidget(zoom_out)
        layout.addLayout(nav_layout)

        self.setLayout(layout)

    def zoom_in(self):
        self.zoom_level *= 1.2
        self.update()

    def zoom_out(self):
        self.zoom_level /= 1.2
        self.update()

    def update_track(self, track_points):
        self.track_points = track_points
        if track_points:
            # Update center to last known position
            self.center_lat = track_points[-1][0]
            self.center_lon = track_points[-1][1]
        self.update()

    def paintEvent(self, event):
        if not self.track_points:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Calculate the scaling factor
        width = self.width()
        height = self.height()

        # Find the bounds of the track
        if len(self.track_points) > 1:
            min_lat = min(p[0] for p in self.track_points)
            max_lat = max(p[0] for p in self.track_points)
            min_lon = min(p[1] for p in self.track_points)
            max_lon = max(p[1] for p in self.track_points)

            # Add some padding
            lat_range = (max_lat - min_lat) * 1.1
            lon_range = (max_lon - min_lon) * 1.1

            # Calculate scaling factors
            lat_scale = height / lat_range if lat_range != 0 else 1
            lon_scale = width / lon_range if lon_range != 0 else 1
            scale = min(lat_scale, lon_scale) * self.zoom_level

            # Draw track
            painter.setPen(QPen(QColor(0, 0, 255), 2))
            first_point = True
            last_x = last_y = 0

            for lat, lon in self.track_points:
                x = (lon - min_lon) * scale + (width - lon_range * scale) / 2
                y = height - ((lat - min_lat) * scale + (height - lat_range * scale) / 2)

                if first_point:
                    first_point = False
                else:
                    painter.drawLine(int(last_x), int(last_y), int(x), int(y))
                last_x, last_y = x, y

            # Draw current position
            current_lat, current_lon = self.track_points[-1]
            x = (current_lon - min_lon) * scale + (width - lon_range * scale) / 2
            y = height - ((current_lat - min_lat) * scale + (height - lat_range * scale) / 2)

            # Draw position marker
            painter.setBrush(QBrush(QColor(255, 0, 0)))
            painter.setPen(QPen(QColor(255, 0, 0), 1))
            painter.drawEllipse(QPointF(x, y), 5, 5)

class MapDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("GPS Track View")
        self.setGeometry(100, 100, 600, 500)

        layout = QVBoxLayout()
        self.map_widget = TrackMapWidget()
        layout.addWidget(self.map_widget)

        self.setLayout(layout)

    def update_position(self, track_points):
        if track_points:
            self.map_widget.update_track(track_points)

class GPSWindow(QMainWindow):
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

        source_layout.addWidget(source_label)
        source_layout.addWidget(self.source_combo)
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
        if not self.map_dialog:
            self.map_dialog = MapDialog(self)
        self.map_dialog.show()
        self.map_dialog.raise_()

    def change_location_source(self, source_name):
        if self.location_manager.set_active_provider(source_name):
            self.clear_track()
            QMessageBox.information(self, "Source Changed",
                                  f"Successfully switched to {source_name}")
        else:
            QMessageBox.warning(self, "Error",
                              f"Failed to switch to {source_name}")

    def toggle_logging(self):
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
        if self.location_manager.active_provider:
            self.location_manager.active_provider.track_history.clear()
        if self.map_dialog:
            self.map_dialog.update_position([])

    def closeEvent(self, event):
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
