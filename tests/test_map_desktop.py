"""Desktop map regressions: drag panning direction/scale and the scale bar length (Qt offscreen, no network)."""
import math

import geo


class _Pos:
    def __init__(self, x, y):
        self._x, self._y = x, y

    def x(self):
        return self._x

    def y(self):
        return self._y


class _Event:
    def __init__(self, x, y):
        self._p = _Pos(x, y)

    def pos(self):
        return self._p


def _drag(widget, dx, dy):
    widget.dragging = True
    widget.last_pos = _Pos(200, 200)
    widget.mouseMoveEvent(_Event(200 + dx, 200 + dy))


def test_drag_down_moves_centre_north_and_keeps_point_under_cursor(map_widget):
    w = map_widget
    w.zoom_level = 14
    w.center_lat, w.center_lon = 51.5, -0.1
    x0, y0 = geo.latlon_to_tile(51.5, -0.1, 14)
    _drag(w, 30, 40)  # content follows the mouse: right and down
    assert w.center_lat > 51.5 and w.center_lon < -0.1
    x1, y1 = geo.latlon_to_tile(w.center_lat, w.center_lon, 14)
    assert math.isclose((x0 - x1) * 256, 30, rel_tol=1e-9)
    assert math.isclose((y0 - y1) * 256, 40, rel_tol=1e-9)


def test_scale_bar_length_matches_label(qapp):
    import mapping
    bar = mapping.ScaleBar()
    for zoom, lat in [(15, 0.0), (12, 45.0), (19, 0.0), (3, 60.0), (17, 30.1)]:
        bar.update_scale(zoom, lat)
        metres, width, label = bar.scale()
        assert label != "0 m"
        assert math.isclose(width * geo.meters_per_pixel(lat, zoom), metres, rel_tol=1e-9)
        assert 0 < width <= bar.width() - 20
