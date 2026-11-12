"""
Read a recorded NMEA log (e.g. the raw serial output of a GPS receiver) into track points with pynmea2.

Uses the same sentences as the live SerialGPSProvider: RMC (position, time, date, speed) from any talker
(GP, GN, GL, ...) and GGA (altitude). A point is made for each RMC with a valid fix (status A); its altitude is the
most recent GGA altitude. Sentences with a bad checksum or unreadable fields are skipped and counted. Ported to
JavaScript in web/static/track_nmea.js.
"""

import datetime
import math
import re

import pynmea2

from track_values import epoch_seconds, make_point

TIME_RE = re.compile(r"^\d{6}(\.\d+)?$", re.ASCII)
DATE_RE = re.compile(r"^\d{6}$", re.ASCII)


def _seconds_of_day(t):
    return t.hour * 3600 + t.minute * 60 + t.second + t.microsecond / 1e6


def _rmc_time(msg):
    """Epoch seconds from an RMC sentence's time and date, or None if either is missing or invalid."""
    raw_t, raw_d = msg.data[0], msg.data[8] if len(msg.data) > 8 else ""
    if not TIME_RE.match(raw_t) or not DATE_RE.match(raw_d):
        return None
    t, d = msg.timestamp, msg.datestamp
    if not isinstance(t, datetime.time) or not isinstance(d, datetime.date):
        return None
    return epoch_seconds(d.year, d.month, d.day, 0, 0, 0) + _seconds_of_day(t)


def parse_nmea(text):
    points, skipped, altitude = [], 0, None
    for raw in text.splitlines():
        line = raw.strip()
        if not line.startswith("$") or line[3:6].upper() not in ("RMC", "GGA"):
            continue
        try:
            msg = pynmea2.parse(line)
            if line[3:6].upper() == "GGA":
                alt = msg.altitude
                altitude = alt if isinstance(alt, float) and math.isfinite(alt) else None
                continue
            if msg.status != "A":
                continue
            if not msg.lat or not msg.lon:
                skipped += 1
                continue
            p = make_point(msg.latitude, msg.longitude, altitude, _rmc_time(msg), None, 0)
        except (pynmea2.ParseError, ValueError, TypeError, IndexError, AttributeError):
            skipped += 1
            continue
        if p:
            points.append(p)
        else:
            skipped += 1
    return {"format": "nmea", "name": "", "points": points, "skipped": skipped}
