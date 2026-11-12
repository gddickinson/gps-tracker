"""
Statistics for a recorded track: distance, time, speed, elevation, heart rate and per-kilometre splits.

compute_stats(points) takes points from track_io.parse_track. Definitions (also shown on the web page):
- distance: sum of great-circle (haversine) distances between consecutive points of the same segment;
- duration: latest timestamp minus earliest; moving time: sum of the time steps whose speed is at least
  MOVING_SPEED_MPS (0.5 m/s = 1.8 km/h), and moving distance the distance covered in those steps;
- max speed: the fastest average over any stretch of at least SPEED_WINDOW_S seconds (smooths GPS jitter);
- elevation gain/loss: climbs and descents counted once they exceed ELEVATION_THRESHOLD_M from the last turning
  point (hysteresis, so noise of a metre or two is not summed up);
- splits: time for each whole kilometre (times interpolated at each kilometre mark) and the remainder.
Ported to JavaScript in web/static/track_stats.js and checked against this module.
"""

import geo

MOVING_SPEED_MPS = 0.5
SPEED_WINDOW_S = 10.0
ELEVATION_THRESHOLD_M = 2.0
SPLIT_M = 1000.0
MPS_TO_KMH = 3.6
MPS_TO_KNOTS = 3600.0 / 1852.0


def cumulative_distance(points):
    """Distance along the track (m) at each point; a segment break adds nothing."""
    cd, total = [0.0] * len(points), 0.0
    for i in range(1, len(points)):
        a, b = points[i - 1], points[i]
        if a["seg"] == b["seg"]:
            total += geo.haversine_m(a["lat"], a["lon"], b["lat"], b["lon"])
        cd[i] = total
    return cd


def _moving(points, cd, moving_speed):
    """(moving_time_s, moving_distance_m) over steps between timed points of the same segment."""
    mt = md = 0.0
    for i in range(1, len(points)):
        a, b = points[i - 1], points[i]
        if a["seg"] != b["seg"] or a["time"] is None or b["time"] is None:
            continue
        dt, dd = b["time"] - a["time"], cd[i] - cd[i - 1]
        if dt > 0 and dd / dt >= moving_speed:
            mt += dt
            md += dd
    return mt, md


def _max_speed(points, cd, window):
    """Fastest average speed (m/s) over a stretch of at least `window` seconds within one segment."""
    idx = [i for i, p in enumerate(points) if p["time"] is not None]
    best, j = None, 0
    for a in range(len(idx)):
        i = idx[a]
        j = max(j, a + 1)
        while j < len(idx) and points[idx[j]]["seg"] == points[i]["seg"] \
                and points[idx[j]]["time"] - points[i]["time"] < window:
            j += 1
        if j >= len(idx) or points[idx[j]]["seg"] != points[i]["seg"]:
            continue
        k = idx[j]
        v = (cd[k] - cd[i]) / (points[k]["time"] - points[i]["time"])
        if best is None or v > best:
            best = v
    return best


def _elevation(points, threshold):
    """(min, max, gain, loss) of elevation, gain/loss with a hysteresis threshold per segment."""
    eles = [p for p in points if p["ele"] is not None]
    if not eles:
        return None, None, None, None
    lo = min(p["ele"] for p in eles)
    hi = max(p["ele"] for p in eles)
    gain = loss = 0.0
    ref, seg = None, None
    for p in eles:
        e = p["ele"]
        if ref is None or p["seg"] != seg:
            ref, seg = e, p["seg"]
            continue
        if e - ref >= threshold:
            gain += e - ref
            ref = e
        elif ref - e >= threshold:
            loss += ref - e
            ref = e
    return lo, hi, gain, loss


def _splits(points, cd, split_m):
    """[{distance_m, time_s}] for each whole split and the remainder, using interpolated crossing times."""
    timed = [i for i, p in enumerate(points) if p["time"] is not None]
    if len(timed) < 2:
        return []
    out, start_t, start_d = [], points[timed[0]]["time"], cd[timed[0]]
    mark = start_d + split_m
    for a, b in zip(timed, timed[1:]):
        da, db = cd[a], cd[b]
        ta, tb = points[a]["time"], points[b]["time"]
        while db > da and db >= mark:
            t = ta + (mark - da) / (db - da) * (tb - ta)
            out.append({"distance_m": split_m, "time_s": t - start_t})
            start_t, start_d, mark = t, mark, mark + split_m
    last = timed[-1]
    rest = cd[last] - start_d
    if rest > 0:
        out.append({"distance_m": rest, "time_s": points[last]["time"] - start_t})
    return out


def compute_stats(points, moving_speed=MOVING_SPEED_MPS, speed_window=SPEED_WINDOW_S,
                  elevation_threshold=ELEVATION_THRESHOLD_M, split_m=SPLIT_M):
    """All statistics for a list of track points (see the module docstring for the definitions)."""
    if not points:
        raise ValueError("No track points.")
    cd = cumulative_distance(points)
    times = [p["time"] for p in points if p["time"] is not None]
    duration = max(times) - min(times) if len(times) >= 2 else None
    mt, md = _moving(points, cd, moving_speed)
    lo, hi, gain, loss = _elevation(points, elevation_threshold)
    hrs = [p["hr"] for p in points if p["hr"] is not None]
    distance = cd[-1]
    return {
        "n_points": len(points),
        "n_segments": len({p["seg"] for p in points}),
        "distance_m": distance,
        "duration_s": duration,
        "moving_time_s": mt if times else None,
        "moving_distance_m": md if times else None,
        "avg_speed_mps": distance / duration if duration else None,
        "moving_speed_mps": md / mt if mt > 0 else None,
        "max_speed_mps": _max_speed(points, cd, speed_window),
        "ele_min_m": lo, "ele_max_m": hi, "ele_gain_m": gain, "ele_loss_m": loss,
        "hr_avg": sum(hrs) / len(hrs) if hrs else None,
        "hr_max": max(hrs) if hrs else None,
        "start_time": min(times) if times else None,
        "bounds": [min(p["lat"] for p in points), min(p["lon"] for p in points),
                   max(p["lat"] for p in points), max(p["lon"] for p in points)],
        "cumdist_m": cd,
        "splits": _splits(points, cd, split_m),
        "settings": {"moving_speed_mps": moving_speed, "speed_window_s": speed_window,
                     "elevation_threshold_m": elevation_threshold, "split_m": split_m},
    }
