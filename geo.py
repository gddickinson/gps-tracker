"""
Qt-free geographic helpers for the GPS tracker.

Web Mercator (slippy-map) projection used by the map widget, the map scale, map panning and great-circle
distances. Imported by mapping.py (desktop map), track_stats.py and the web interface; ported to JavaScript in
web/static/geo.js (checked against this module by web/static/track_engine.test.mjs).
"""

import math

EARTH_RADIUS_M = 6371000.0          # mean Earth radius used for haversine distances
EQUATOR_M_PER_PX_Z0 = 156543.03392  # ground metres per pixel at zoom 0 on the equator (256 px tiles)
TILE_SIZE = 256
MAX_LAT = 85.0511287798066          # Web Mercator latitude limit


def clamp_lat(lat):
    """Clamp a latitude to the range Web Mercator can show."""
    return max(-MAX_LAT, min(MAX_LAT, lat))


def latlon_to_tile(lat, lon, zoom):
    """Latitude/longitude (degrees) to fractional tile coordinates (x, y) at a zoom level."""
    n = 2.0 ** zoom
    lat_rad = math.radians(clamp_lat(lat))
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def tile_to_latlon(x, y, zoom):
    """Fractional tile coordinates back to latitude/longitude (degrees)."""
    n = 2.0 ** zoom
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def pan(lat, lon, dx_px, dy_px, zoom):
    """New map centre after dragging the map content by (dx_px, dy_px) screen pixels.

    Dragging right/down moves the content right/down, so the centre moves west/north.
    """
    x, y = latlon_to_tile(lat, lon, zoom)
    return tile_to_latlon(x - dx_px / TILE_SIZE, y - dy_px / TILE_SIZE, zoom)


def meters_per_pixel(lat, zoom):
    """Ground resolution (metres per screen pixel) at a latitude and zoom level."""
    return EQUATOR_M_PER_PX_Z0 * math.cos(math.radians(lat)) / (2 ** zoom)


def nice_scale(lat, zoom, target_px=100):
    """Scale bar for the map: a round distance near target_px pixels long.

    Returns (metres, width_px, label); the bar width matches the label exactly. Round distances are 1, 2 or 5
    times a power of ten.
    """
    raw = meters_per_pixel(lat, zoom) * target_px
    if raw <= 0:
        return 0.0, 0.0, "0 m"
    power = 10.0 ** math.floor(math.log10(raw))
    nice = power
    for step in (1, 2, 5, 10):
        if step * power <= raw:
            nice = step * power
    width = nice / meters_per_pixel(lat, zoom)
    if nice >= 1000:
        label = f"{nice / 1000:g} km"
    else:
        label = f"{nice:g} m"
    return nice, width, label


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two points given in degrees."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def path_length_m(points):
    """Total length in metres of a path given as a sequence of (lat, lon) pairs."""
    total = 0.0
    for (a_lat, a_lon), (b_lat, b_lon) in zip(points, points[1:]):
        total += haversine_m(a_lat, a_lon, b_lat, b_lon)
    return total
