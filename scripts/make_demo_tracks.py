"""Write the synthetic demo tracks used by the web page and the JS-vs-Python tests (tests/fixtures/).

The route is invented: a figure-of-eight of about 6 km around 0 deg N, 0 deg E (open sea, "Null Island"), with a made-up hill
profile, heart rate and a one-minute stop, so it shows nobody's real location. The same route is written as GPX
(two segments), TCX, CSV in this app's own log format (speed in knots, as logged by gui.py) and as a raw NMEA log
(RMC + GGA, with a lost fix and a corrupted sentence). Run: python scripts/make_demo_tracks.py
"""
import math
import random
from functools import reduce
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIX = ROOT / "tests" / "fixtures"
START = 1767254400  # 2026-01-01T08:00:00Z
M_PER_DEG = 111195.0


def curve(u):
    """Figure of eight in metres east/north of 0 N, 0 E."""
    return 1000 * math.sin(u), 480 * math.sin(2 * u)


def route():
    """List of (t, lat, lon, ele, hr, seg) every 5 s, moving along the curve at a realistic running speed."""
    rnd = random.Random(7)
    # arc-length table so the distance covered matches the chosen speed
    us = [k / 20000 * 2 * math.pi for k in range(20001)]
    arc = [0.0]
    for u0, u1 in zip(us, us[1:]):
        (x0, y0), (x1, y1) = curve(u0), curve(u1)
        arc.append(arc[-1] + math.hypot(x1 - x0, y1 - y0))
    length = arc[-1]
    pts, t, s, seg, k = [], 0.0, 0.0, 0, 0
    while s < length:
        while arc[k + 1] < s:
            k += 1
        u = us[k] + (s - arc[k]) / (arc[k + 1] - arc[k]) * (us[k + 1] - us[k])
        x, y = curve(u)
        climb = max(0.0, math.sin(u))
        ele = 12 + 38 * climb ** 2 + rnd.uniform(-0.6, 0.6)
        speed = 3.3 - 0.6 * climb * math.cos(u) + rnd.uniform(-0.15, 0.15)   # m/s: slower up the hill, faster down
        hr = round(132 + 24 * climb + rnd.uniform(-2, 2))
        lat = (y + rnd.uniform(-1.5, 1.5)) / M_PER_DEG
        lon = (x + rnd.uniform(-1.5, 1.5)) / M_PER_DEG
        pts.append((START + t, round(lat, 7), round(lon, 7), round(ele, 1), hr, seg))
        if seg == 0 and s >= length / 2:   # one-minute stop half-way: new segment
            t += 60
            seg = 1
        t += 5
        s += 5 * speed
    return pts


def iso(t, frac=False):
    days, rem = divmod(int(t), 86400)
    import datetime
    d = datetime.datetime(1970, 1, 1) + datetime.timedelta(days=days, seconds=rem)
    return d.strftime("%Y-%m-%dT%H:%M:%S") + (".000" if frac else "") + "Z"


def gpx(pts):
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<gpx version="1.1" creator="gps-tracker demo (synthetic)" xmlns="http://www.topografix.com/GPX/1/1"'
           ' xmlns:gpxtpx="http://www.garmin.com/xmlschemas/TrackPointExtension/v1">',
           '  <metadata><name>Synthetic demo loop (Null Island)</name></metadata>',
           '  <trk><name>Synthetic demo loop</name>']
    for seg in (0, 1):
        out.append('    <trkseg>')
        for t, lat, lon, ele, hr, s in pts:
            if s == seg:
                out.append(f'      <trkpt lat="{lat}" lon="{lon}"><ele>{ele}</ele><time>{iso(t)}</time>'
                           f'<extensions><gpxtpx:TrackPointExtension><gpxtpx:hr>{hr}</gpxtpx:hr>'
                           f'</gpxtpx:TrackPointExtension></extensions></trkpt>')
        out.append('    </trkseg>')
    out += ['  </trk>', '</gpx>', '']
    return "\n".join(out)


def tcx(pts):
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<TrainingCenterDatabase xmlns="http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2">',
           '  <Activities><Activity Sport="Running"><Id>' + iso(pts[0][0]) + '</Id>']
    for seg in (0, 1):
        sp = [p for p in pts if p[5] == seg]
        out.append(f'    <Lap StartTime="{iso(sp[0][0])}"><Track>')
        for t, lat, lon, ele, hr, s in sp:
            out.append(f'      <Trackpoint><Time>{iso(t, True)}</Time><Position><LatitudeDegrees>{lat}</LatitudeDegrees>'
                       f'<LongitudeDegrees>{lon}</LongitudeDegrees></Position><AltitudeMeters>{ele}</AltitudeMeters>'
                       f'<HeartRateBpm><Value>{hr}</Value></HeartRateBpm></Trackpoint>')
        out.append('    </Track></Lap>')
    out += ['  </Activity></Activities>', '</TrainingCenterDatabase>', '']
    return "\n".join(out)


def csv_log(pts):
    """The CSV columns gui.py writes when logging (timestamp, latitude, longitude, speed in knots, ...)."""
    out = ["timestamp,latitude,longitude,speed,altitude,satellites,source"]
    for k, (t, lat, lon, ele, hr, s) in enumerate(pts):
        nxt = pts[min(k + 1, len(pts) - 1)]
        dist = math.hypot((nxt[1] - lat) * M_PER_DEG, (nxt[2] - lon) * M_PER_DEG)
        knots = dist / max(nxt[0] - t, 1) * 3600 / 1852
        out.append(f"{iso(t)[:-1]}.250000,{lat},{lon},{knots:.1f},{ele},9,Serial GPS (/dev/tty.usbmodem101)")
    return "\r\n".join(out) + "\r\n"


def nmea_sentence(body):
    return "$%s*%02X" % (body, reduce(lambda a, c: a ^ ord(c), body, 0))


def dm(value, lat):
    deg = int(abs(value))
    minutes = (abs(value) - deg) * 60
    hemi = ("N" if value >= 0 else "S") if lat else ("E" if value >= 0 else "W")
    return (f"{deg:02d}{minutes:07.4f}" if lat else f"{deg:03d}{minutes:07.4f}"), hemi


def nmea(pts):
    out = []
    for k, (t, lat, lon, ele, hr, s) in enumerate(pts[:120]):
        hms = iso(t)[11:19].replace(":", "") + ".00"
        date = iso(t)[8:10] + iso(t)[5:7] + iso(t)[2:4]
        la, ns = dm(lat, True)
        lo, ew = dm(lon, False)
        out.append(nmea_sentence(f"GPGGA,{hms},{la},{ns},{lo},{ew},1,09,0.9,{ele},M,17.0,M,,"))
        status = "V" if k == 40 else "A"
        talker = "GN" if k >= 60 else "GP"
        line = nmea_sentence(f"{talker}RMC,{hms},{status},{la},{ns},{lo},{ew},6.2,90.0,{date},,,A")
        if k == 50:
            line = line[:-2] + ("00" if line[-2:] != "00" else "11")   # corrupted checksum
        out.append(line)
        if k == 10:
            out.append("$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74")
    return "\r\n".join(out) + "\r\n"


def main():
    FIX.mkdir(parents=True, exist_ok=True)
    pts = route()
    (FIX / "demo_route.gpx").write_text(gpx(pts))
    (FIX / "demo_route.tcx").write_text(tcx(pts))
    (FIX / "demo_log.csv").write_bytes(csv_log(pts).encode())
    (FIX / "demo_log.nmea").write_bytes(nmea(pts).encode())
    print(f"{len(pts)} points written to {FIX}")


if __name__ == "__main__":
    main()
