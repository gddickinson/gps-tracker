"""geo.py: projection round trips and known distances."""
import math

import geo


def test_tile_round_trip():
    for lat, lon, z in [(0, 0, 1), (51.5, -0.12, 15), (-33.9, 151.2, 10), (85, 179.9, 3)]:
        x, y = geo.latlon_to_tile(lat, lon, z)
        lat2, lon2 = geo.tile_to_latlon(x, y, z)
        assert math.isclose(lat, lat2, abs_tol=1e-9) and math.isclose(lon, lon2, abs_tol=1e-9)


def test_origin_tile_is_centre():
    assert geo.latlon_to_tile(0, 0, 1) == (1.0, 1.0)


def test_haversine_one_degree_on_equator():
    assert math.isclose(geo.haversine_m(0, 0, 0, 1), math.radians(1) * geo.EARTH_RADIUS_M, rel_tol=1e-12)


def test_path_length():
    pts = [(0, 0), (0, 1), (1, 1)]
    assert math.isclose(geo.path_length_m(pts), geo.haversine_m(0, 0, 0, 1) + geo.haversine_m(0, 1, 1, 1))
    assert geo.path_length_m([]) == 0 and geo.path_length_m([(1, 2)]) == 0
