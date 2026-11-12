"""
Read recorded tracks: GPX, TCX, CSV (including this app's own log format) and NMEA logs.

parse_track(text, filename) returns {"format", "name", "points", "skipped"}, where each point is a dict
{lat, lon, ele, time, hr, seg} (see track_values.make_point); time is seconds since the epoch, seg numbers the
track segments (no distance is counted across a segment break). Used by the web interface; ported to JavaScript in
web/static/track_io.js. NMEA logs are read by track_nmea.py with pynmea2, like the live serial provider.
"""

import csv
import io
import xml.etree.ElementTree as ET

from track_values import make_point, parse_number, parse_time

FORMATS = ("gpx", "tcx", "csv", "nmea")
CSV_COLUMNS = {
    "lat": ("latitude", "lat"),
    "lon": ("longitude", "lon", "lng", "long"),
    "ele": ("altitude", "elevation", "ele", "alt"),
    "time": ("timestamp", "time", "datetime", "date_time"),
    "hr": ("heart_rate", "heartrate", "hr"),
}


def _local(tag):
    return tag.rsplit("}", 1)[-1]


def _children(el, name):
    return [c for c in el if _local(c.tag) == name]


def _child(el, name):
    for c in el:
        if _local(c.tag) == name:
            return c
    return None


def _text(el):
    return (el.text or "").strip() if el is not None else ""


def _find_hr(el):
    """Heart rate from a GPX point's extensions (any element named 'hr', e.g. gpxtpx:hr)."""
    ext = _child(el, "extensions")
    if ext is None:
        return None
    for d in ext.iter():
        if _local(d.tag) == "hr":
            return parse_number(_text(d))
    return None


def _xml_root(text):
    try:
        return ET.fromstring(text.lstrip("\ufeff").strip())
    except ET.ParseError as e:
        raise ValueError(f"Not valid XML: {e}") from None


def parse_gpx(text):
    root = _xml_root(text)
    if _local(root.tag) != "gpx":
        raise ValueError("Not a GPX file (no <gpx> root element).")
    points, skipped, seg = [], 0, 0
    segments = [s for trk in _children(root, "trk") for s in _children(trk, "trkseg")]
    lists = [_children(s, "trkpt") for s in segments] + [_children(r, "rtept") for r in _children(root, "rte")]
    for pts in lists:
        if not pts:
            continue
        for el in pts:
            p = make_point(parse_number(el.get("lat")), parse_number(el.get("lon")),
                           parse_number(_text(_child(el, "ele"))), parse_time(_text(_child(el, "time"))),
                           _find_hr(el), seg)
            if p:
                points.append(p)
            else:
                skipped += 1
        seg += 1
    name = ""
    for holder in (_child(root, "metadata"), *_children(root, "trk"), *_children(root, "rte")):
        if holder is not None and _text(_child(holder, "name")):
            name = _text(_child(holder, "name"))
            break
    return {"format": "gpx", "name": name, "points": points, "skipped": skipped}


def parse_tcx(text):
    root = _xml_root(text)
    if _local(root.tag) != "TrainingCenterDatabase":
        raise ValueError("Not a TCX file (no <TrainingCenterDatabase> root element).")
    points, skipped, seg, name = [], 0, 0, ""
    for acts in _children(root, "Activities"):
        for act in _children(acts, "Activity"):
            name = name or " ".join(x for x in (act.get("Sport", ""), _text(_child(act, "Id"))) if x)
            for lap in _children(act, "Lap"):
                for track in _children(lap, "Track"):
                    tps = _children(track, "Trackpoint")
                    for tp in tps:
                        pos = _child(tp, "Position")
                        hr = _child(tp, "HeartRateBpm")
                        p = make_point(parse_number(_text(_child(pos, "LatitudeDegrees"))) if pos is not None else None,
                                       parse_number(_text(_child(pos, "LongitudeDegrees"))) if pos is not None else None,
                                       parse_number(_text(_child(tp, "AltitudeMeters"))),
                                       parse_time(_text(_child(tp, "Time"))),
                                       parse_number(_text(_child(hr, "Value"))) if hr is not None else None, seg)
                        if p:
                            points.append(p)
                        else:
                            skipped += 1
                    if tps:
                        seg += 1
    return {"format": "tcx", "name": name, "points": points, "skipped": skipped}


def _delimiter(header_line):
    counts = {d: header_line.count(d) for d in (",", ";", "\t")}
    best = max(counts.values())
    return next(d for d in (",", ";", "\t") if counts[d] == best)


def parse_csv(text):
    text = text.lstrip("\ufeff")
    lines = text.splitlines()
    if not lines:
        raise ValueError("The CSV file is empty.")
    rows = list(csv.reader(io.StringIO(text), delimiter=_delimiter(lines[0])))
    if not rows:
        raise ValueError("The CSV file is empty.")
    header = [h.strip().lower() for h in rows[0]]
    col = {}
    for key, names in CSV_COLUMNS.items():
        col[key] = next((header.index(n) for n in names if n in header), None)
    if col["lat"] is None or col["lon"] is None:
        raise ValueError("CSV needs latitude and longitude columns (e.g. 'latitude,longitude').")

    def cell(row, key):
        i = col[key]
        return row[i] if i is not None and i < len(row) else ""

    points, skipped = [], 0
    for row in rows[1:]:
        if not any(c.strip() for c in row):
            continue
        p = make_point(parse_number(cell(row, "lat")), parse_number(cell(row, "lon")),
                       parse_number(cell(row, "ele")), parse_time(cell(row, "time")),
                       parse_number(cell(row, "hr")), 0)
        if p:
            points.append(p)
        else:
            skipped += 1
    return {"format": "csv", "name": "", "points": points, "skipped": skipped}


def detect_format(text, filename=""):
    """'gpx', 'tcx', 'csv' or 'nmea' from the file extension, else from the content."""
    ext = filename.rsplit(".", 1)[-1].lower() if "." in filename else ""
    if ext in FORMATS:
        return ext
    head = text.lstrip("\ufeff").lstrip()[:2000]
    if head.startswith("<"):
        return "tcx" if "TrainingCenterDatabase" in head else "gpx"
    if head.startswith("$"):
        return "nmea"
    return "csv"


def parse_track(text, filename=""):
    """Parse a track file's text; raises ValueError with a readable message for unusable files."""
    fmt = detect_format(text, filename)
    if fmt == "nmea":
        from track_nmea import parse_nmea
        result = parse_nmea(text)
    else:
        result = {"gpx": parse_gpx, "tcx": parse_tcx, "csv": parse_csv}[fmt](text)
    if not result["points"]:
        raise ValueError(f"No usable track points found in this {fmt.upper()} file.")
    return result
