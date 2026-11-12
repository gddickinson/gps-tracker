// Strict number and ISO 8601 time parsing for the track readers: a port of track_values.py (same rules, same results).
const NUMBER_RE = /^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$/;
const ISO_TIME_RE = /^(\d{4})-(\d{2})-(\d{2})[T ](\d{2}):(\d{2}):(\d{2})(\.\d+)?(Z|[+-]\d{2}:?\d{2})?$/i;
const DAYS_IN_MONTH = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
const WS = '\\t\\n\\x0b\\x0c\\r\\x1c-\\x20\\x85\\xa0\\u1680\\u2000-\\u200a\\u2028\\u2029\\u202f\\u205f\\u3000';   // Python's str.isspace()
const PY_SPACE = new RegExp(`^[${WS}]+|[${WS}]+$`, 'g');
export const pyStrip = s => String(s).replace(PY_SPACE, '');
export const pyLstrip = s => String(s).replace(new RegExp(`^[${WS}]+`), '');

export function parseNumber(text) {
  if (text === null || text === undefined) return null;
  const s = pyStrip(text);
  if (!NUMBER_RE.test(s)) return null;
  const v = Number(s);
  return Number.isFinite(v) ? v : null;
}

export const isLeap = y => y % 4 === 0 && (y % 100 !== 0 || y % 400 === 0);
export function validDate(y, m, d) {
  if (m < 1 || m > 12 || d < 1) return false;
  return d <= (m === 2 && isLeap(y) ? 29 : DAYS_IN_MONTH[m - 1]);
}

export function daysFromCivil(y, m, d) {
  y -= m <= 2 ? 1 : 0;
  const era = Math.floor((y >= 0 ? y : y - 399) / 400);
  const yoe = y - era * 400;
  const doy = Math.floor((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5) + d - 1;
  const doe = yoe * 365 + Math.floor(yoe / 4) - Math.floor(yoe / 100) + doy;
  return era * 146097 + doe - 719468;
}

export const epochSeconds = (y, mo, d, h, mi, s, frac = 0, offset = 0) =>
  daysFromCivil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s - offset + frac;

export function parseTime(text) {
  if (text === null || text === undefined) return null;
  const s = pyStrip(text);
  const m = ISO_TIME_RE.exec(s);
  if (!m) return s ? parseNumber(s) : null;
  const [y, mo, d, h, mi, sec] = m.slice(1, 7).map(Number);
  if (!validDate(y, mo, d) || h > 23 || mi > 59 || sec > 60) return null;
  const frac = m[7] ? Number('0' + m[7]) : 0;
  const zone = (m[8] || 'Z').toUpperCase();
  let offset = 0;
  if (zone !== 'Z') {
    const digits = zone.slice(1).replace(':', ''), hh = +digits.slice(0, 2), mm = +digits.slice(2);
    if (hh > 23 || mm > 59) return null;
    offset = (hh * 3600 + mm * 60) * (zone[0] === '+' ? 1 : -1);
  }
  return epochSeconds(y, mo, d, h, mi, sec, frac, offset);
}

export function makePoint(lat, lon, ele = null, time = null, hr = null, seg = 0) {
  if (lat === null || lon === null || !(lat >= -90 && lat <= 90) || !(lon >= -180 && lon <= 180)) return null;
  return { lat, lon, ele, time, hr, seg };
}
