// Track readers for GPX, TCX, CSV and NMEA: a port of track_io.py (same points, same skipped counts).
// parseTrack(text, filename) -> { format, name, points: [{lat, lon, ele, time, hr, seg}], skipped }.
import { parseNumber, parseTime, makePoint, pyStrip, pyLstrip } from './track_values.js';
import { parseXml, children, child, iter } from './xml_lite.js';
import { parseNmea } from './track_nmea.js';

export const FORMATS = ['gpx', 'tcx', 'csv', 'nmea'];
const CSV_COLUMNS = {
  lat: ['latitude', 'lat'], lon: ['longitude', 'lon', 'lng', 'long'], ele: ['altitude', 'elevation', 'ele', 'alt'],
  time: ['timestamp', 'time', 'datetime', 'date_time'], hr: ['heart_rate', 'heartrate', 'hr'],
};
const text = el => (el ? pyStrip(el.text) : '');

function findHr(el) {
  const ext = child(el, 'extensions');
  if (!ext) return null;
  for (const d of iter(ext)) if (d.name === 'hr') return parseNumber(text(d));
  return null;
}

export function parseGpx(src) {
  const root = parseXml(src);
  if (root.name !== 'gpx') throw new Error('Not a GPX file (no <gpx> root element).');
  const points = [];
  let skipped = 0, seg = 0;
  const segments = children(root, 'trk').flatMap(t => children(t, 'trkseg'));
  const lists = [...segments.map(s => children(s, 'trkpt')), ...children(root, 'rte').map(r => children(r, 'rtept'))];
  for (const pts of lists) {
    if (!pts.length) continue;
    for (const el of pts) {
      const p = makePoint(parseNumber(el.attrs.lat ?? null), parseNumber(el.attrs.lon ?? null),
        parseNumber(text(child(el, 'ele'))), parseTime(text(child(el, 'time'))), findHr(el), seg);
      if (p) points.push(p); else skipped++;
    }
    seg++;
  }
  let name = '';
  for (const h of [child(root, 'metadata'), ...children(root, 'trk'), ...children(root, 'rte')]) {
    if (h && text(child(h, 'name'))) { name = text(child(h, 'name')); break; }
  }
  return { format: 'gpx', name, points, skipped };
}

export function parseTcx(src) {
  const root = parseXml(src);
  if (root.name !== 'TrainingCenterDatabase') throw new Error('Not a TCX file (no <TrainingCenterDatabase> root element).');
  const points = [];
  let skipped = 0, seg = 0, name = '';
  for (const acts of children(root, 'Activities')) for (const act of children(acts, 'Activity')) {
    name = name || [act.attrs.Sport || '', text(child(act, 'Id'))].filter(Boolean).join(' ');
    for (const lap of children(act, 'Lap')) for (const track of children(lap, 'Track')) {
      const tps = children(track, 'Trackpoint');
      for (const tp of tps) {
        const pos = child(tp, 'Position'), hr = child(tp, 'HeartRateBpm');
        const p = makePoint(pos ? parseNumber(text(child(pos, 'LatitudeDegrees'))) : null,
          pos ? parseNumber(text(child(pos, 'LongitudeDegrees'))) : null,
          parseNumber(text(child(tp, 'AltitudeMeters'))), parseTime(text(child(tp, 'Time'))),
          hr ? parseNumber(text(child(hr, 'Value'))) : null, seg);
        if (p) points.push(p); else skipped++;
      }
      if (tps.length) seg++;
    }
  }
  return { format: 'tcx', name, points, skipped };
}

function delimiter(header) {
  const cands = [',', ';', '\t'], counts = cands.map(d => header.split(d).length - 1), best = Math.max(...counts);
  return cands[counts.indexOf(best)];
}

// RFC 4180 reader matching Python's csv.reader for ordinary files (quoted fields, doubled quotes, CR/LF).
export function readCsv(src, delim) {
  const rows = [];
  let row = [], field = '', quoted = false, i = 0, atStart = true;
  while (i < src.length) {
    const c = src[i];
    if (quoted) {
      if (c === '"') { if (src[i + 1] === '"') { field += '"'; i++; } else quoted = false; } else field += c;
    } else if (c === '"' && atStart) { quoted = true; atStart = false; }
    else if (c === delim) { row.push(field); field = ''; atStart = true; i++; continue; }
    else if (c === '\n' || c === '\r') {
      row.push(field); rows.push(row); row = []; field = ''; atStart = true;
      if (c === '\r' && src[i + 1] === '\n') i++;
      i++; continue;
    } else { field += c; atStart = false; }
    i++;
  }
  if (field !== '' || row.length || quoted) { row.push(field); rows.push(row); }
  return rows;
}

export function parseCsv(src) {
  src = src.replace(/^\ufeff+/, '');
  const first = src.split(/\r\n|\r|\n/)[0];
  if (!src.length) throw new Error('The CSV file is empty.');
  const rows = readCsv(src, delimiter(first));
  const header = rows[0].map(h => pyStrip(h).toLowerCase()), col = {};
  for (const [key, names] of Object.entries(CSV_COLUMNS)) {
    const n = names.find(x => header.includes(x));
    col[key] = n === undefined ? null : header.indexOf(n);
  }
  if (col.lat === null || col.lon === null) throw new Error("CSV needs latitude and longitude columns (e.g. 'latitude,longitude').");
  const cell = (row, key) => (col[key] !== null && col[key] < row.length ? row[col[key]] : '');
  const points = [];
  let skipped = 0;
  for (const row of rows.slice(1)) {
    if (!row.some(c => pyStrip(c))) continue;
    const p = makePoint(parseNumber(cell(row, 'lat')), parseNumber(cell(row, 'lon')), parseNumber(cell(row, 'ele')),
      parseTime(cell(row, 'time')), parseNumber(cell(row, 'hr')), 0);
    if (p) points.push(p); else skipped++;
  }
  return { format: 'csv', name: '', points, skipped };
}

export function detectFormat(src, filename = '') {
  const ext = filename.includes('.') ? filename.split('.').pop().toLowerCase() : '';
  if (FORMATS.includes(ext)) return ext;
  const head = pyLstrip(src.replace(/^\ufeff+/, '')).slice(0, 2000);
  if (head.startsWith('<')) return head.includes('TrainingCenterDatabase') ? 'tcx' : 'gpx';
  if (head.startsWith('$')) return 'nmea';
  return 'csv';
}

export function parseTrack(src, filename = '') {
  const fmt = detectFormat(src, filename);
  const result = { gpx: parseGpx, tcx: parseTcx, csv: parseCsv, nmea: parseNmea }[fmt](src);
  if (!result.points.length) throw new Error(`No usable track points found in this ${fmt.toUpperCase()} file.`);
  return result;
}
