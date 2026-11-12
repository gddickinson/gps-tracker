"""SerialGPSProvider / GUI regressions with a fake serial port (no hardware, no network)."""
import types

import pytest

import providers

GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
RMC = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"
GN_RMC = "$GNRMC,123520,A,4807.038,N,00000.000,E,000.0,084.4,230394,003.1,W*78\r\n"
GN_GGA = "$GNGGA,123520,4807.038,N,00000.000,E,1,09,0.9,0.0,M,46.9,M,,*50\r\n"


class FakeSerial:
    """Minimal pyserial stand-in: readline() returns queued lines, in_waiting counts unread bytes."""
    def __init__(self, lines):
        self.lines = [ln.encode("ascii") for ln in lines]
        self.is_open = True

    @property
    def in_waiting(self):
        return sum(len(ln) for ln in self.lines)

    def readline(self):
        return self.lines.pop(0) if self.lines else b""

    def close(self):
        self.is_open = False


def make_provider(monkeypatch, lines):
    fake = FakeSerial(lines)
    monkeypatch.setattr(providers.serial, "Serial", lambda *a, **k: fake)
    return providers.SerialGPSProvider("/dev/fake"), fake


def test_altitude_and_satellites_arrive_with_the_position(monkeypatch):
    p, _ = make_provider(monkeypatch, [GGA, RMC])
    data = p.get_location()
    assert data["latitude"] == pytest.approx(48.1173) and data["longitude"] == pytest.approx(11.516666666)
    assert float(data["altitude"]) == 545.4 and int(data["satellites"]) == 8
    assert float(data["speed"]) == 22.4


def test_buffered_sentences_are_all_read(monkeypatch):
    p, fake = make_provider(monkeypatch, [GGA, RMC] * 5)
    p.get_location()
    assert fake.lines == []          # no growing backlog: each update reads everything waiting
    assert len(p.track_history) == 5


def test_bad_line_is_skipped_without_closing_the_port(monkeypatch):
    p, fake = make_provider(monkeypatch, ["$GPRMC,garbage*00\r\n", GGA, RMC])
    data = p.get_location()
    assert fake.is_open and data["latitude"] == pytest.approx(48.1173)


def test_multi_gnss_talker_and_zero_longitude(monkeypatch):
    p, _ = make_provider(monkeypatch, [GN_GGA, GN_RMC])
    data = p.get_location()
    assert data["longitude"] == 0.0 and float(data["speed"]) == 0.0 and float(data["altitude"]) == 0.0
    assert p.track_history == [(pytest.approx(48.1173), 0.0)]


def test_gui_shows_zero_speed_and_altitude():
    import gui

    class Label:
        def __init__(self):
            self.text = "N/A"

        def setText(self, t):
            self.text = t

    fake = types.SimpleNamespace(labels={k: Label() for k in (
        "time", "date", "latitude", "longitude", "speed", "track angle", "fix quality",
        "satellites", "altitude", "hdop", "source", "distance")})
    gui.GPSWindow._update_labels(fake, {"speed": 3.0, "altitude": 12.0, "latitude": 1.0, "longitude": 1.0})
    gui.GPSWindow._update_labels(fake, {"speed": 0.0, "altitude": 0.0, "latitude": 0.0, "longitude": 0.0})
    assert fake.labels["speed"].text == "0.0 knots"
    assert fake.labels["altitude"].text == "0.0m"
    assert fake.labels["latitude"].text == "0.000000"
