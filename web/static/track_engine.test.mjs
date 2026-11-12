// Node test: the JS port (geo, track_values, track_io, track_nmea, track_stats) returns what the Python modules return
// for every fixture file and value table in tests/fixtures/track_expected.json (made by scripts/make_track_fixture.py).
// Run: node web/static/track_engine.test.mjs
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import * as geo from './geo.js';
import { parseNumber, parseTime } from './track_values.js';
import { parseTrack, detectFormat } from './track_io.js';
import { parseNmea } from './track_nmea.js';
import { computeStats } from './track_stats.js';

const FIX = join(dirname(fileURLToPath(import.meta.url)), '..', '..', 'tests', 'fixtures');
const exp = JSON.parse(readFileSync(join(FIX, 'track_expected.json'), 'utf8'));
let checks = 0, fails = 0;

function close(a, b, path) {
  checks++;
  const bad = msg => { fails++; if (fails < 30) console.log(`FAIL ${path}: ${msg}`); };
  if (b === null || typeof b !== 'object') {
    if (typeof b === 'number' && typeof a === 'number') {
      if (!(Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b)))) bad(`${a} != ${b}`);
    } else if (a !== b) bad(`${JSON.stringify(a)} != ${JSON.stringify(b)}`);
    return;
  }
  if (a === null || typeof a !== 'object') return bad(`${JSON.stringify(a)} != object`);
  const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
  if (Array.isArray(b) && a.length !== b.length) return bad(`length ${a.length} != ${b.length}`);
  for (const k of keys) close(a[k], b[k], `${path}.${k}`);
}

const orError = fn => { try { return fn(); } catch (e) { return 'error'; } };

for (const [name, want] of Object.entries(exp.files)) {
  const text = readFileSync(join(FIX, name), 'utf8');
  const parsed = parseTrack(text, name);
  close(parsed, want.parsed, `${name}.parsed`);
  close(computeStats(parsed.points), want.stats, `${name}.stats`);
}
const demo = parseTrack(readFileSync(join(FIX, 'demo_route.gpx'), 'utf8'), 'demo_route.gpx').points;
close(computeStats(demo, { movingSpeed: 2.0, speedWindow: 30.0, elevationThreshold: 5.0, splitM: 1609.344 }), exp.custom_stats, 'custom');
for (const [s, v] of exp.numbers) close(parseNumber(s), v, `number ${JSON.stringify(s)}`);
for (const [s, v] of exp.times) close(parseTime(s), v, `time ${JSON.stringify(s)}`);
for (const [s, v] of exp.nmea) close(parseNmea(s), v, `nmea ${JSON.stringify(s.slice(0, 40))}`);
for (const [t, n, v] of exp.bad_files) close(orError(() => parseTrack(t, n)), v, `bad file ${JSON.stringify(t)}`);
for (const [t, n, v] of exp.detect) close(detectFormat(t, n), v, `detect ${n}`);
for (const g of exp.geo) {
  const [lat, lon, z] = g.in, tile = geo.latlonToTile(lat, lon, z);
  close(tile, g.tile, `tile ${g.in}`);
  close(geo.tileToLatlon(tile[0], tile[1], z), g.back, `back ${g.in}`);
  close(geo.metersPerPixel(lat, z), g.mpp, `mpp ${g.in}`);
  close(geo.niceScale(lat, z), g.scale, `scale ${g.in}`);
  close(geo.pan(lat, lon, 37, -25, z), g.pan, `pan ${g.in}`);
  close(geo.haversineM(lat, lon, lat + 0.3, lon - 0.7), g.hav, `hav ${g.in}`);
}
if (fails) { console.log(`${fails} of ${checks} checks failed`); process.exit(1); }
console.log(`ok ${checks} checks: JS track engine matches Python`);
