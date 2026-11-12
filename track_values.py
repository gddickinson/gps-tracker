"""
Strict value parsing shared by the track readers (track_io.py, track_nmea.py).

Numbers and ISO 8601 times are parsed with explicit rules (not float()/datetime.fromisoformat) so the JavaScript
port in web/static/track_values.js gives exactly the same results. Times become seconds since 1970-01-01 UTC; a time
without a zone is read as UTC (only differences between times are used for statistics).
"""

import math
import re

NUMBER_RE = re.compile(r"^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$", re.ASCII)
ISO_TIME_RE = re.compile(
    r"^(\d{4})-(\d{2})-(\d{2})[T ](\d{2}):(\d{2}):(\d{2})(\.\d+)?(Z|[+-]\d{2}:?\d{2})?$", re.IGNORECASE | re.ASCII)
DAYS_IN_MONTH = (31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)


def parse_number(text):
    """A finite decimal number, or None for empty/invalid text (no 'nan', 'inf', '1_000', hex...)."""
    if text is None:
        return None
    s = str(text).strip()
    if not NUMBER_RE.match(s):
        return None
    v = float(s)
    return v if math.isfinite(v) else None


def is_leap(year):
    return year % 4 == 0 and (year % 100 != 0 or year % 400 == 0)


def valid_date(y, m, d):
    if not 1 <= m <= 12 or d < 1:
        return False
    return d <= (29 if m == 2 and is_leap(y) else DAYS_IN_MONTH[m - 1])


def days_from_civil(y, m, d):
    """Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's algorithm)."""
    y -= m <= 2
    era = (y if y >= 0 else y - 399) // 400
    yoe = y - era * 400
    doy = (153 * (m + (-3 if m > 2 else 9)) + 2) // 5 + d - 1
    doe = yoe * 365 + yoe // 4 - yoe // 100 + doy
    return era * 146097 + doe - 719468


def epoch_seconds(y, mo, d, h, mi, s, frac=0.0, offset_s=0):
    """Seconds since the Unix epoch (UTC) for a date and time with an offset from UTC in seconds."""
    whole = days_from_civil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s - offset_s
    return whole + frac


def parse_time(text):
    """ISO 8601 date-time ('2024-05-01T08:00:00.5Z', '... 08:00:00', '...+02:00') or a plain number of epoch
    seconds, to seconds since the epoch. None if empty or invalid."""
    if text is None:
        return None
    s = str(text).strip()
    m = ISO_TIME_RE.match(s)
    if not m:
        return parse_number(s) if s else None
    y, mo, d, h, mi, sec = (int(g) for g in m.groups()[:6])
    if not valid_date(y, mo, d) or h > 23 or mi > 59 or sec > 60:
        return None
    frac = float("0" + m.group(7)) if m.group(7) else 0.0
    zone = (m.group(8) or "Z").upper()
    offset = 0
    if zone != "Z":
        digits = zone[1:].replace(":", "")
        hh, mm = int(digits[:2]), int(digits[2:])
        if hh > 23 or mm > 59:
            return None
        offset = (hh * 3600 + mm * 60) * (1 if zone[0] == "+" else -1)
    return epoch_seconds(y, mo, d, h, mi, sec, frac, offset)


def make_point(lat, lon, ele=None, time=None, hr=None, seg=0):
    """A track point dict, or None when the position is missing or out of range."""
    if lat is None or lon is None or not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        return None
    return {"lat": lat, "lon": lon, "ele": ele, "time": time, "hr": hr, "seg": seg}
