// NMEA log reader: a port of track_nmea.py, which uses pynmea2. Sentence pattern, checksum, degree/minute conversion,
// time and date rules follow pynmea2 1.19 so both give the same points for well-formed logs.
import { epochSeconds, makePoint, parseNumber, pyStrip } from './track_values.js';

// pynmea2's sentence regex (IGNORECASE): optional '$', talker+type, data, optional *HH checksum.
const SENTENCE = /^\s*\$?((P\w{3}|\w{2}\w{2}Q,\w{3}|\w{2}\w{3},)([^*]*))(?:[*]([A-F0-9]{2}))?\s*[\r\n]*$/i;
const TIME_RE = /^\d{6}(\.\d+)?$/, DATE_RE = /^\d{6}$/;

function dmToSd(dm) {          // pynmea2.nmea_utils.dm_to_sd
  if (!dm || dm === '0') return 0;
  const r = /^(\d+)(\d\d\.\d+)$/.exec(dm);
  if (!r) throw new Error(`invalid coordinate ${dm}`);
  return Number(r[1]) + Number(r[2]) / 60;
}

const signed = (dm, dir, pos, neg) => { const sd = dmToSd(dm); return dir === pos ? sd : dir === neg ? -sd : 0; };

function rmcTime(f) {
  const t = f[0] ?? '', d = f[8] ?? '';
  if (!TIME_RE.test(t) || !DATE_RE.test(d)) return null;
  const hh = +t.slice(0, 2), mi = +t.slice(2, 4), ss = +t.slice(4, 6);
  const us = t.length > 6 ? Math.trunc(Number(t.slice(6)) * 1000000) : 0;
  if (hh > 23 || mi > 59 || ss > 59 || us > 999999) return null;
  const day = +d.slice(0, 2), mon = +d.slice(2, 4), yy = +d.slice(4, 6), year = yy < 69 ? 2000 + yy : 1900 + yy;
  const dim = [31, (year % 4 === 0 && (year % 100 !== 0 || year % 400 === 0)) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
  if (mon < 1 || mon > 12 || day < 1 || day > dim[mon - 1]) return null;
  return epochSeconds(year, mon, day, 0, 0, 0) + (hh * 3600 + mi * 60 + ss + us / 1e6);
}

// Parse one sentence like pynmea2.parse: returns the data fields, or throws on a bad pattern or checksum.
function sentenceFields(line) {
  const m = SENTENCE.exec(line);
  if (!m) throw new Error('could not parse data');
  if (!/^\w{2}\w{3},$/.test(m[2])) throw new Error('not a talker sentence');   // proprietary/query: no RMC/GGA fields
  if (m[4] !== undefined) {
    let cs = 0;
    for (const ch of m[1]) cs ^= ch.codePointAt(0);
    if (parseInt(m[4], 16) !== cs) throw new Error('checksum does not match');
  }
  return m[3].split(',');
}

// pynmea2 float field: None when empty, the raw text (not a number) when float() fails.
const pyFloatField = v => (v === undefined || v === '' ? null : parseNumber(v));

export function parseNmea(src) {
  const points = [];
  let skipped = 0, altitude = null;
  for (const raw of src.split(/\r\n|\r|\n/)) {
    const line = pyStrip(raw), kind = line.slice(3, 6).toUpperCase();
    if (!line.startsWith('$') || (kind !== 'RMC' && kind !== 'GGA')) continue;
    let p;
    try {
      const f = sentenceFields(line);
      if (kind === 'GGA') { altitude = pyFloatField(f[8]); continue; }
      if ((f[1] ?? '') !== 'A') continue;
      if (!f[2] || !f[4]) { skipped++; continue; }
      p = makePoint(signed(f[2], f[3] ?? '', 'N', 'S'), signed(f[4], f[5] ?? '', 'E', 'W'), altitude, rmcTime(f), null, 0);
    } catch (e) { skipped++; continue; }
    if (p) points.push(p); else skipped++;
  }
  return { format: 'nmea', name: '', points, skipped };
}
