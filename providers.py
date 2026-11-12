"""
Location provider classes for GPS tracker.

Provides abstract base class and concrete implementations for
serial GPS and IP-based geolocation data sources.
"""

import time
import platform
import logging
from datetime import datetime
from abc import ABC, abstractmethod

import serial
import pynmea2
import requests

logger = logging.getLogger(__name__)


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
        self._state = self._empty_state()
        self.connect()

    def connect(self):
        """Establish connection to serial port"""
        try:
            if not self.serial_port or not self.serial_port.is_open:
                self.serial_port = serial.Serial(self.port, self.baudrate)
                logger.info("Connected to %s", self.port)
        except serial.SerialException as e:
            logger.error("Failed to connect to %s: %s", self.port, e)
            self.serial_port = None

    MAX_LINES_PER_READ = 200  # bound one update even if the device floods the port

    def _empty_state(self):
        return {
            'latitude': None, 'longitude': None, 'altitude': None,
            'speed': None, 'satellites': None, 'fix_quality': None,
            'time': None, 'date': None, 'track_angle': None,
            'hdop': None, 'source': self.name
        }

    def _apply_sentence(self, line):
        """Merge one NMEA sentence (RMC or GGA from any talker: GP, GN, GL, ...) into the current state."""
        line = line.strip()
        kind = line[3:6] if line.startswith('$') else ''
        if kind not in ('RMC', 'GGA'):
            return
        try:
            msg = pynmea2.parse(line)
            state = self._state
            if kind == 'RMC':
                state['time'] = msg.timestamp
                state['date'] = msg.datestamp
                if msg.status == 'A':  # Valid fix
                    lat, lon = msg.latitude, msg.longitude
                    state['latitude'], state['longitude'] = lat, lon
                    state['speed'] = msg.spd_over_grnd
                    state['track_angle'] = msg.true_course
                    if lat is not None and lon is not None:
                        self.track_history.append((lat, lon))
                        if len(self.track_history) > 1000:
                            self.track_history.pop(0)
                else:  # fix lost: don't keep showing the old position as current
                    state['latitude'] = state['longitude'] = state['speed'] = None
            else:
                state['fix_quality'] = msg.gps_qual
                state['satellites'] = msg.num_sats
                state['altitude'] = msg.altitude
                state['hdop'] = msg.horizontal_dil
        except (pynmea2.ParseError, ValueError, TypeError, AttributeError) as e:
            logger.debug("Skipping bad NMEA sentence %r: %s", line, e)

    def get_location(self):
        """
        Read every NMEA sentence waiting on the serial port and merge them.
        Returns: dict with the latest position (RMC) and fix details (GGA), or None if the port fails.
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.connect()
            if not self.serial_port:
                return None

        try:
            lines = [self.serial_port.readline()]
            while self.serial_port.in_waiting and len(lines) < self.MAX_LINES_PER_READ:
                lines.append(self.serial_port.readline())
        except (serial.SerialException, OSError) as e:
            logger.error("Error reading GPS: %s", e)
            self.close()
            return None

        for raw in lines:
            self._apply_sentence(raw.decode('ascii', errors='replace'))
        return dict(self._state)

    def close(self):
        """Close serial port connection"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                logger.info("Closed %s", self.port)
            except serial.SerialException:
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

        if self.cached_data and (current_time - self.last_update) < self.cache_duration:
            return self.cached_data

        try:
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

                if data['latitude'] is not None and data['longitude'] is not None:
                    self.track_history.append((data['latitude'], data['longitude']))
                    if len(self.track_history) > 1000:
                        self.track_history.pop(0)

                self.cached_data = data
                self.last_update = current_time
                return data
        except requests.RequestException as e:
            logger.error("Error getting IP location: %s", e)
        return None

    def close(self):
        """Cleanup resources (no-op for IP provider)"""
        pass

    @property
    def name(self):
        return self._name


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
        self.providers['ip'] = IPLocationProvider()
        self.detect_serial_devices()

    def detect_serial_devices(self):
        """Detect available serial GPS devices"""
        system = platform.system()
        if system == 'Darwin':
            self._detect_macos_devices()
        elif system == 'Linux':
            self._detect_linux_devices()
        elif system == 'Windows':
            self._detect_windows_devices()

    def _detect_macos_devices(self):
        """Detect GPS devices on macOS"""
        import glob as glob_module
        patterns = [
            '/dev/tty.usbmodem*',
            '/dev/tty.usbserial*',
            '/dev/tty.SLAB_*',
            '/dev/tty.GPS*'
        ]

        for pattern in patterns:
            for port in glob_module.glob(pattern):
                try:
                    test_serial = serial.Serial(port, 115200, timeout=1)
                    test_serial.close()

                    with serial.Serial(port, 115200, timeout=1) as ser:
                        for _ in range(5):
                            line = ser.readline().decode('ascii', errors='replace').strip()
                            if line.startswith('$G'):
                                provider = SerialGPSProvider(port)
                                self.providers[f'serial_{port}'] = provider
                                logger.info("Found GPS device on %s", port)
                                break

                except (serial.SerialException, OSError):
                    continue

    def _detect_linux_devices(self):
        """Detect GPS devices on Linux"""
        import glob as glob_module
        patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/ttyS*',
            '/dev/gps*'
        ]

        for pattern in patterns:
            for port in glob_module.glob(pattern):
                try:
                    with serial.Serial(port, 115200, timeout=1) as ser:
                        for _ in range(5):
                            line = ser.readline().decode('ascii', errors='replace').strip()
                            if line.startswith('$G'):
                                provider = SerialGPSProvider(port)
                                self.providers[f'serial_{port}'] = provider
                                logger.info("Found GPS device on %s", port)
                                break
                except (serial.SerialException, OSError):
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
                    logger.info("Found GPS device on %s", port.device)
                    continue

                with serial.Serial(port.device, 115200, timeout=1) as ser:
                    for _ in range(5):
                        line = ser.readline().decode('ascii', errors='replace').strip()
                        if line.startswith('$G'):
                            provider = SerialGPSProvider(port.device)
                            self.providers[f'serial_{port.device}'] = provider
                            logger.info("Found GPS device on %s", port.device)
                            break
            except (serial.SerialException, OSError):
                continue

    def refresh_devices(self):
        """Refresh the list of available devices"""
        for provider in self.providers.values():
            if isinstance(provider, SerialGPSProvider):
                provider.close()

        self.providers = {k: v for k, v in self.providers.items()
                         if not isinstance(v, SerialGPSProvider)}
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
