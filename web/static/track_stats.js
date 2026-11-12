// Track statistics: a port of track_stats.py (same definitions; see that module's docstring or the About page).
import { haversineM } from './geo.js';

export const MOVING_SPEED_MPS = 0.5, SPEED_WINDOW_S = 10.0, ELEVATION_THRESHOLD_M = 2.0, SPLIT_M = 1000.0;
export const MPS_TO_KMH = 3.6, MPS_TO_KNOTS = 3600 / 1852;

export function cumulativeDistance(points) {
  const cd = new Array(points.length).fill(0);
  let total = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    if (a.seg === b.seg) total += haversineM(a.lat, a.lon, b.lat, b.lon);
    cd[i] = total;
  }
  return cd;
}

function moving(points, cd, minSpeed) {
  let mt = 0, md = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    if (a.seg !== b.seg || a.time === null || b.time === null) continue;
    const dt = b.time - a.time, dd = cd[i] - cd[i - 1];
    if (dt > 0 && dd / dt >= minSpeed) { mt += dt; md += dd; }
  }
  return [mt, md];
}

function maxSpeed(points, cd, win) {
  const idx = [];
  points.forEach((p, i) => { if (p.time !== null) idx.push(i); });
  let best = null, j = 0;
  for (let a = 0; a < idx.length; a++) {
    const i = idx[a];
    j = Math.max(j, a + 1);
    while (j < idx.length && points[idx[j]].seg === points[i].seg && points[idx[j]].time - points[i].time < win) j++;
    if (j >= idx.length || points[idx[j]].seg !== points[i].seg) continue;
    const k = idx[j], v = (cd[k] - cd[i]) / (points[k].time - points[i].time);
    if (best === null || v > best) best = v;
  }
  return best;
}

function elevation(points, threshold) {
  const eles = points.filter(p => p.ele !== null);
  if (!eles.length) return [null, null, null, null];
  let lo = Infinity, hi = -Infinity, gain = 0, loss = 0, ref = null, seg = null;
  for (const p of eles) { lo = Math.min(lo, p.ele); hi = Math.max(hi, p.ele); }
  for (const p of eles) {
    const e = p.ele;
    if (ref === null || p.seg !== seg) { ref = e; seg = p.seg; continue; }
    if (e - ref >= threshold) { gain += e - ref; ref = e; } else if (ref - e >= threshold) { loss += ref - e; ref = e; }
  }
  return [lo, hi, gain, loss];
}

function splits(points, cd, splitM) {
  const timed = [];
  points.forEach((p, i) => { if (p.time !== null) timed.push(i); });
  if (timed.length < 2) return [];
  const out = [];
  let startT = points[timed[0]].time, startD = cd[timed[0]], mark = startD + splitM;
  for (let n = 1; n < timed.length; n++) {
    const a = timed[n - 1], b = timed[n], da = cd[a], db = cd[b], ta = points[a].time, tb = points[b].time;
    while (db > da && db >= mark) {
      const t = ta + (mark - da) / (db - da) * (tb - ta);
      out.push({ distance_m: splitM, time_s: t - startT });
      startT = t; startD = mark; mark = mark + splitM;
    }
  }
  const last = timed[timed.length - 1], rest = cd[last] - startD;
  if (rest > 0) out.push({ distance_m: rest, time_s: points[last].time - startT });
  return out;
}

export function computeStats(points, { movingSpeed = MOVING_SPEED_MPS, speedWindow = SPEED_WINDOW_S,
  elevationThreshold = ELEVATION_THRESHOLD_M, splitM = SPLIT_M } = {}) {
  if (!points.length) throw new Error('No track points.');
  const mn = a => a.reduce((x, y) => (y < x ? y : x)), mx = a => a.reduce((x, y) => (y > x ? y : x));
  const cd = cumulativeDistance(points);
  const times = points.filter(p => p.time !== null).map(p => p.time);
  const duration = times.length >= 2 ? mx(times) - mn(times) : null;
  const [mt, md] = moving(points, cd, movingSpeed);
  const [lo, hi, gain, loss] = elevation(points, elevationThreshold);
  const hrs = points.filter(p => p.hr !== null).map(p => p.hr);
  const distance = cd[cd.length - 1];
  let hrSum = 0;
  for (const h of hrs) hrSum += h;
  const lats = points.map(p => p.lat), lons = points.map(p => p.lon);
  return {
    n_points: points.length,
    n_segments: new Set(points.map(p => p.seg)).size,
    distance_m: distance,
    duration_s: duration,
    moving_time_s: times.length ? mt : null,
    moving_distance_m: times.length ? md : null,
    avg_speed_mps: duration ? distance / duration : null,
    moving_speed_mps: mt > 0 ? md / mt : null,
    max_speed_mps: maxSpeed(points, cd, speedWindow),
    ele_min_m: lo, ele_max_m: hi, ele_gain_m: gain, ele_loss_m: loss,
    hr_avg: hrs.length ? hrSum / hrs.length : null,
    hr_max: hrs.length ? mx(hrs) : null,
    start_time: times.length ? mn(times) : null,
    bounds: [mn(lats), mn(lons), mx(lats), mx(lons)],
    cumdist_m: cd,
    splits: splits(points, cd, splitM),
    settings: { moving_speed_mps: movingSpeed, speed_window_s: speedWindow, elevation_threshold_m: elevationThreshold, split_m: splitM },
  };
}
