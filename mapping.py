"""
Map display components for GPS tracker.

Provides tile management with caching, scale bar widget,
interactive map widget with pan/zoom, and the map dialog.
"""

import os
import math
import time
import logging
import sqlite3
from datetime import datetime

import requests
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QDialog
from PyQt6.QtCore import Qt, QPointF, QRectF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush, QPixmap, QPainterPath, QFont

logger = logging.getLogger(__name__)

# Default tile server configuration
TILE_SERVERS = {
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


class TileManager:
    """Manages map tiles with local caching and rate limiting"""
    def __init__(self, cache_dir="map_cache"):
        self.cache_dir = cache_dir
        self.tile_servers = dict(TILE_SERVERS)
        self.current_server = 'OpenStreetMap'
        self.download_timeout = 3
        self.last_download_time = {}
        self.min_download_interval = 0.1
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
            logger.error("Cache error: %s", e)

        # If not in cache, try to download
        server_info = self.tile_servers[self.current_server]
        headers = {'User-Agent': server_info['user_agent']}

        self._respect_rate_limit(self.current_server)
        success = self._try_download_tile(server_info['url'], x, y, zoom, headers, tile_key)
        if success:
            return success

        if 'backup_urls' in server_info:
            for backup_url in server_info['backup_urls']:
                self._respect_rate_limit(self.current_server)
                success = self._try_download_tile(backup_url, x, y, zoom, headers, tile_key)
                if success:
                    return success

        return self._get_default_tile(x, y, zoom)

    def _try_download_tile(self, url_template, x, y, zoom, headers, tile_key):
        """Attempt to download a tile from a specific URL"""
        url = url_template.format(x=x, y=y, z=zoom)
        try:
            response = requests.get(url, timeout=self.download_timeout, headers=headers)
            if response.status_code == 200:
                try:
                    with sqlite3.connect(self.db_path) as conn:
                        conn.execute('INSERT OR REPLACE INTO tiles VALUES (?, ?, ?)',
                                   (tile_key, response.content,
                                    datetime.now().isoformat()))
                except sqlite3.Error as e:
                    logger.error("Cache write error: %s", e)

                pixmap = QPixmap()
                if pixmap.loadFromData(response.content):
                    return pixmap
        except (requests.RequestException, requests.Timeout) as e:
            logger.error("Download error for %s: %s", url, e)
        return None

    def _get_default_tile(self, x, y, zoom):
        """Create an informative default tile"""
        pixmap = QPixmap(256, 256)
        pixmap.fill(QColor(240, 240, 240))
        painter = QPainter(pixmap)

        painter.setPen(QPen(QColor(200, 200, 200)))
        painter.drawRect(0, 0, 255, 255)

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

        meters_per_pixel = 156543.03392 * math.cos(math.radians(self.latitude)) / (2 ** self.zoom_level)
        scale_width = 100
        distance = meters_per_pixel * scale_width

        if distance >= 1000:
            distance = round(distance / 1000) * 1000
            label = f"{distance/1000:.0f} km"
        else:
            distance = round(distance / 100) * 100
            label = f"{distance:.0f} m"

        painter.setPen(QPen(Qt.GlobalColor.black, 2))
        y = self.height() - 5
        painter.drawLine(10, y, 10 + scale_width, y)
        painter.drawLine(10, y-5, 10, y+5)
        painter.drawLine(10 + scale_width, y-5, 10 + scale_width, y+5)

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

        self.zoom_level = 15
        self.center_lat = 0
        self.center_lon = 0
        self.tracks = []
        self.dragging = False
        self.last_pos = None

        self.tile_manager = TileManager()

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.status_message = QLabel()
        self.status_message.setStyleSheet("color: red;")
        layout.addWidget(self.status_message)

        controls = QHBoxLayout()

        self.map_type = QComboBox()
        self.map_type.addItems(self.tile_manager.tile_servers.keys())
        self.map_type.currentTextChanged.connect(self.change_map_type)
        controls.addWidget(QLabel("Map Type:"))
        controls.addWidget(self.map_type)

        refresh_button = QPushButton("Refresh Map")
        refresh_button.clicked.connect(self.force_refresh)
        controls.addWidget(refresh_button)

        zoom_in = QPushButton("+")
        zoom_out = QPushButton("-")
        zoom_in.clicked.connect(self.zoom_in)
        zoom_out.clicked.connect(self.zoom_out)
        controls.addWidget(zoom_in)
        controls.addWidget(zoom_out)

        self.scale_bar = ScaleBar()
        controls.addWidget(self.scale_bar)

        controls.addStretch()
        layout.addLayout(controls)

        self.status_bar = QLabel()
        layout.addWidget(self.status_bar)

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

        center_x, center_y = self.latlon_to_pixel(self.center_lat, self.center_lon)
        tile_x = int(center_x)
        tile_y = int(center_y)

        pixel_x = int((center_x - tile_x) * 256)
        pixel_y = int((center_y - tile_y) * 256)

        for dx in range(-2, 3):
            for dy in range(-2, 3):
                x = tile_x + dx
                y = tile_y + dy

                if 0 <= x < 2**self.zoom_level and 0 <= y < 2**self.zoom_level:
                    tile = self.tile_manager.get_tile(x, y, self.zoom_level)
                    draw_x = int(self.width()/2 + (dx*256) - pixel_x)
                    draw_y = int(self.height()/2 + (dy*256) - pixel_y)
                    painter.drawPixmap(draw_x, draw_y, tile)

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

            last_lat, last_lon = self.tracks[-1]
            x, y = self.latlon_to_pixel(last_lat, last_lon)
            screen_x = int(self.width()/2 + (x - center_x) * 256)
            screen_y = int(self.height()/2 + (y - center_y) * 256)

            painter.setBrush(QBrush(QColor(255, 0, 0)))
            painter.drawEllipse(QPointF(screen_x, screen_y), 5, 5)

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
            self.map_widget.update_tracks(track_points)
            current_lat, current_lon = track_points[-1]
            self.map_widget.set_center(current_lat, current_lon)
