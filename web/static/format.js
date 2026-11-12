// Number formatting for the track pages (units: metres, seconds, m/s in; readable text out).
import { MPS_TO_KMH, MPS_TO_KNOTS } from './track_stats.js';

export const UNITS = {
  kmh: { label: 'km/h', f: v => v * MPS_TO_KMH },
  knots: { label: 'knots', f: v => v * MPS_TO_KNOTS },    // the desktop app shows speed in knots (NMEA)
};

export function dist(m) {
  if (m === null || m === undefined) return '–';
  return m < 1000 ? `${Math.round(m)} m` : `${(m / 1000).toFixed(m < 100000 ? 2 : 1)} km`;
}

export function dur(s) {
  if (s === null || s === undefined) return '–';
  const t = Math.round(s), h = Math.floor(t / 3600), m = Math.floor(t / 60) % 60, sec = t % 60;
  const pad = n => String(n).padStart(2, '0');
  return h ? `${h}:${pad(m)}:${pad(sec)}` : `${m}:${pad(sec)}`;
}

export function speed(v, unit = 'kmh') {
  if (v === null || v === undefined) return '–';
  return `${UNITS[unit].f(v).toFixed(1)} ${UNITS[unit].label}`;
}

// Pace (time per kilometre) for a speed in m/s.
export const pace = v => (v ? `${dur(1000 / v)} /km` : '–');

export const num = (v, digits = 0, suffix = '') => (v === null || v === undefined ? '–' : `${v.toFixed(digits)}${suffix}`);

export function when(epoch) {
  if (epoch === null || epoch === undefined) return '–';
  const d = new Date(epoch * 1000);
  return Number.isNaN(d.getTime()) ? '–' : d.toISOString().replace('T', ' ').replace(/\.\d+Z$/, ' UTC').replace('Z', ' UTC');
}
