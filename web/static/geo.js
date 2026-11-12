// Web Mercator projection, map scale and great-circle distances: a port of geo.py (checked by track_engine.test.mjs).
export const EARTH_RADIUS_M = 6371000.0;
export const EQUATOR_M_PER_PX_Z0 = 156543.03392;
export const TILE_SIZE = 256;
export const MAX_LAT = 85.0511287798066;
const rad = d => d * Math.PI / 180;

export const clampLat = lat => Math.max(-MAX_LAT, Math.min(MAX_LAT, lat));

export function latlonToTile(lat, lon, zoom) {
  const n = 2 ** zoom, r = rad(clampLat(lat));
  return [(lon + 180) / 360 * n, (1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * n];
}

export function tileToLatlon(x, y, zoom) {
  const n = 2 ** zoom;
  return [Math.atan(Math.sinh(Math.PI * (1 - 2 * y / n))) * 180 / Math.PI, x / n * 360 - 180];
}

export const metersPerPixel = (lat, zoom) => EQUATOR_M_PER_PX_Z0 * Math.cos(rad(lat)) / 2 ** zoom;

// Round 1/2/5 x 10^n distance near targetPx pixels: [metres, widthPx, label] (geo.nice_scale).
export function niceScale(lat, zoom, targetPx = 100) {
  const raw = metersPerPixel(lat, zoom) * targetPx;
  if (!(raw > 0)) return [0, 0, '0 m'];
  const power = 10 ** Math.floor(Math.log10(raw));
  let nice = power;
  for (const step of [1, 2, 5, 10]) if (step * power <= raw) nice = step * power;
  const width = nice / metersPerPixel(lat, zoom);
  return [nice, width, nice >= 1000 ? `${+(nice / 1000).toPrecision(6)} km` : `${+nice.toPrecision(6)} m`];
}

export function haversineM(lat1, lon1, lat2, lon2) {
  const p1 = rad(lat1), p2 = rad(lat2), dp = p2 - p1, dl = rad(lon2 - lon1);
  const a = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1, Math.sqrt(a)));
}

// Map centre after dragging the content by (dx, dy) screen pixels (geo.pan).
export function pan(lat, lon, dx, dy, zoom) {
  const [x, y] = latlonToTile(lat, lon, zoom);
  return tileToLatlon(x - dx / TILE_SIZE, y - dy / TILE_SIZE, zoom);
}
