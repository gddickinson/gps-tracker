"""
Main GUI window for GPS tracker application.

Contains the GPSWindow class which manages the main UI,
data collection, logging, and map visualization.
"""

import sys
import csv
import logging
from datetime import datetime

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                           QVBoxLayout, QHBoxLayout, QLabel, QGridLayout,
                           QPushButton, QFileDialog, QTabWidget,
                           QComboBox, QMessageBox)
from PyQt6.QtCore import QTimer

from providers import LocationProviderManager
from mapping import MapDialog

logger = logging.getLogger(__name__)


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

        self.source_combo.addItems(self.location_manager.get_provider_names())
        self.source_combo.currentTextChanged.connect(self.change_location_source)

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
        self.timer.start(1000)

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

        self.source_combo.clear()
        self.source_combo.addItems(self.location_manager.get_provider_names())

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
            self._update_labels(data)
            self._update_map()
            self._log_data(data)

    def _update_labels(self, data):
        """Update the GPS data labels from location data"""
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
                self.labels['track angle'].setText(f"{angle_val:.1f}\u00b0")
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

    def _update_map(self):
        """Update the map dialog if visible"""
        if self.map_dialog and self.map_dialog.isVisible():
            self.map_dialog.update_position(self.location_manager.get_track_history())

    def _log_data(self, data):
        """Log data to CSV if logging is enabled"""
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
