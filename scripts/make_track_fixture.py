"""Write tests/fixtures/track_expected.json: what the Python modules (geo, track_values, track_io, track_nmea,
track_stats) return for the fixture files and value tables. web/static/track_engine.test.mjs checks that the JS port
returns the same, and tests/test_track.py checks that this file is current.

Run: python scripts/make_track_fixture.py
"""
import json
import sys
from functools import reduce
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import geo  # noqa: E402
import track_io  # noqa: E402
import track_stats  # noqa: E402
from track_nmea import parse_nmea  # noqa: E402
from track_values import parse_number, parse_time  # noqa: E402

FIX = ROOT / "tests" / "fixtures"
OUT = FIX / "track_expected.json"
FILES = ["demo_route.gpx", "demo_route.tcx", "demo_log.csv", "demo_log.nmea", "edge_cases.gpx", "edge_cases.csv"]

NUMBERS = ["1", "-2.5", "+3.", ".5", " 7 ", "1e3", "-1.5E-2", "", " ", "abc", "nan", "inf", "1_000", "0x10", "1,5",
           "1.2.3", "--1", "1e", "١٢", "\t42\n"]
TIMES = ["2024-05-01T08:00:00Z", "2024-05-01T08:00:00.123456Z", "2024-05-01 08:00:00", "2024-05-01T08:00:00+02:00",
         "2024-05-01T08:00:00-0530", "2024-02-29T00:00:00Z", "2023-02-29T00:00:00Z", "2024-05-01T24:00:00Z",
         "2024-05-01T08:00:60Z", "1969-12-31T23:59:59Z", "1600-03-01T00:00:00Z", "2024-05-01t08:00:00z",
         "1700000000", "1700000000.25", "", "yesterday", "2024-05-01", "2024-05-01T08:00:00+24:00",
         "2025-01-10T10:43:12.864665"]


def cs(body):
    return "$%s*%02X" % (body, reduce(lambda a, c: a ^ ord(c), body, 0))


NMEA_CASES = [
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),
    cs("GPRMC,123519.25,A,4807.038,S,01131.000,W,,,230394,,"),
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,,003.1,W"),          # no date
    cs("GPRMC,253519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),    # hour 25
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,310294,003.1,W"),    # 31 Feb
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230368,003.1,W"),    # year 2068
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230369,003.1,W"),    # year 1969
    cs("GPRMC,123519,A,,,01131.000,E,022.4,084.4,230394,003.1,W"),             # no latitude
    cs("GPRMC,123519,A,48x7.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),    # bad latitude
    cs("GPRMC,123519,A,4807.038,X,01131.000,E,022.4,084.4,230394,003.1,W"),    # odd direction -> 0
    cs("GPRMC,123519,A,9907.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),    # out of range
    cs("GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),    # no fix
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00",   # bad checksum
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W",      # no checksum
    cs("gnrmc,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").lower(),
    cs("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,abc,M,46.9,M,,") + "\n" +
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),
    cs("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-12.5,M,46.9,M,,") + "\r\n" +
    cs("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),
    cs("PGRMC,123519,A,1,2"),
    "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W",       # no '$': ignored
]
GEO = [(0, 0, 1), (51.5, -0.12, 15), (-33.9, 151.2, 10), (89.9, 179.9, 3), (30.1, -97.7, 17), (-60, -170, 19)]
BAD_FILES = [("", "x.csv"), ("a,b\n1,2\n", "x.csv"), ("<gpx><trk>", "x.gpx"), ("<foo/>", "x.gpx"),
             ("<gpx></gpx>", "x.gpx"), ("<gpx/>", "x.tcx"), ("$GPGSV,1,1,00*79\n", "x.nmea"),
             ("<gpx>&nope;</gpx>", "x.gpx"), ("latitude,longitude\n91,0\n", "x.csv")]


def parse_or_error(text, name):
    try:
        return track_io.parse_track(text, name)
    except ValueError:
        return "error"


def build():
    files = {}
    for name in FILES:
        text = (FIX / name).read_text(encoding="utf-8")
        parsed = track_io.parse_track(text, name)
        files[name] = {"parsed": parsed, "stats": track_stats.compute_stats(parsed["points"])}
    custom = track_stats.compute_stats(files["demo_route.gpx"]["parsed"]["points"], moving_speed=2.0,
                                       speed_window=30.0, elevation_threshold=5.0, split_m=1609.344)
    return {
        "files": files,
        "custom_stats": custom,
        "numbers": [[s, parse_number(s)] for s in NUMBERS],
        "times": [[s, parse_time(s)] for s in TIMES],
        "nmea": [[s, parse_nmea(s)] for s in NMEA_CASES],
        "bad_files": [[t, n, parse_or_error(t, n)] for t, n in BAD_FILES],
        "detect": [[t, n, track_io.detect_format(t, n)] for t, n in [
            ("<gpx/>", ""), (" <TrainingCenterDatabase/>", "a.xml"), ("$GPRMC", "log.txt"), ("lat,lon", ""),
            ("x", "A.GPX"), ("<gpx/>", "track.CSV")]],
        "geo": [{"in": [lat, lon, z], "tile": geo.latlon_to_tile(lat, lon, z),
                 "back": geo.tile_to_latlon(*geo.latlon_to_tile(lat, lon, z), z),
                 "mpp": geo.meters_per_pixel(lat, z), "scale": geo.nice_scale(lat, z),
                 "pan": geo.pan(lat, lon, 37, -25, z), "hav": geo.haversine_m(lat, lon, lat + 0.3, lon - 0.7)}
                for lat, lon, z in GEO],
    }


def main():
    OUT.write_text(json.dumps(build(), indent=None, separators=(",", ":")) + "\n")
    print(f"wrote {OUT} ({OUT.stat().st_size / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
