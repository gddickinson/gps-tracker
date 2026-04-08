"""
Smoke tests for GPS tracker providers and tile manager.
Tests basic instantiation and interface compliance without requiring hardware.
"""

from providers import (
    LocationProvider, IPLocationProvider, SerialGPSProvider,
    LocationProviderManager
)


def test_ip_provider_instantiation():
    """IPLocationProvider should instantiate without errors."""
    provider = IPLocationProvider()
    assert provider.name == "IP Geolocation"
    assert provider.track_history == []
    assert provider.cached_data is None


def test_ip_provider_has_required_interface():
    """IPLocationProvider must implement all abstract methods."""
    provider = IPLocationProvider()
    assert hasattr(provider, 'get_location')
    assert hasattr(provider, 'close')
    assert hasattr(provider, 'name')


def test_ip_provider_close_is_noop():
    """Closing IP provider should not raise."""
    provider = IPLocationProvider()
    provider.close()  # should not raise


def test_serial_provider_interface():
    """SerialGPSProvider must implement all abstract methods."""
    assert hasattr(SerialGPSProvider, 'get_location')
    assert hasattr(SerialGPSProvider, 'close')
    assert hasattr(SerialGPSProvider, 'name')


def test_location_provider_manager_instantiation():
    """Manager should instantiate and always have IP provider."""
    manager = LocationProviderManager()
    assert 'ip' in manager.providers
    names = manager.get_provider_names()
    assert "IP Geolocation" in names
    manager.close()


def test_manager_set_active_provider():
    """Setting active provider to IP Geolocation should succeed."""
    manager = LocationProviderManager()
    result = manager.set_active_provider("IP Geolocation")
    assert result is True
    assert manager.active_provider is not None
    manager.close()


def test_manager_set_invalid_provider():
    """Setting a nonexistent provider should fail gracefully."""
    manager = LocationProviderManager()
    result = manager.set_active_provider("Nonexistent Provider")
    assert result is False
    manager.close()


def test_manager_get_track_history_empty():
    """Track history should be empty initially."""
    manager = LocationProviderManager()
    manager.set_active_provider("IP Geolocation")
    history = manager.get_track_history()
    assert history == []
    manager.close()


def test_manager_get_location_no_active():
    """Getting location with no active provider should return None."""
    manager = LocationProviderManager()
    manager.active_provider = None
    result = manager.get_location()
    assert result is None
    manager.close()


if __name__ == "__main__":
    test_ip_provider_instantiation()
    test_ip_provider_has_required_interface()
    test_ip_provider_close_is_noop()
    test_serial_provider_interface()
    test_location_provider_manager_instantiation()
    test_manager_set_active_provider()
    test_manager_set_invalid_provider()
    test_manager_get_track_history_empty()
    test_manager_get_location_no_active()
    print("All tests passed!")
