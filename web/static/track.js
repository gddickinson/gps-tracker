// Track page: open a GPX/TCX/CSV/NMEA file, parse it and compute the statistics in the browser (JS ports of
// track_io.py and track_stats.py), then show the numbers, the route map, the elevation profile and the splits.
// On the local app, "Check with Python" sends the file to the Python modules and compares every number.
import { route, app, api, isStatic, esc, $, needsServer } from './core.js';
import { parseTrack } from './track_io.js';
import { computeStats } from './track_stats.js';
import { mountMap } from './track_map.js';
import { elevationProfile, splitsTable } from './track_charts.js';
import { dist, dur, speed, pace, num, when, UNITS } from './format.js';

let current = null;   // { name, text, parsed, stats, error }
let unit = 'kmh';

function settingsFrom(info) {
  const read = (id, fallback, min) => { const v = Number($(id).value); return Number.isFinite(v) && v >= min ? v : fallback; };
  return {
    movingSpeed: read('#set-move', info.settings.moving_speed_mps * 3.6, 0) / 3.6,
    speedWindow: read('#set-window', info.settings.speed_window_s, 1),
    elevationThreshold: read('#set-ele', info.settings.elevation_threshold_m, 0.1),
    splitM: Number($('#set-split').value),
  };
}

function tile(value, label) { return `<div class="tile"><b>${value}</b><span>${label}</span></div>`; }

function renderResult() {
  const out = $('#result');
  if (!current) { out.innerHTML = ''; return; }
  if (current.error) { out.innerHTML = `<div class="panel"><h3>${esc(current.name)}</h3><p class="err">${esc(current.error)}</p></div>`; return; }
  const { parsed, stats: s } = current;
  const hasTime = s.duration_s !== null, hasEle = s.ele_min_m !== null, hasHr = s.hr_avg !== null;
  out.innerHTML = `<div class="panel"><div class="filehead"><h3>${esc(current.name)}</h3>
      <span class="tag a">${parsed.format.toUpperCase()}</span>${parsed.name ? `<span class="tag">${esc(parsed.name)}</span>` : ''}
      <span class="tag">${s.n_points} points</span><span class="tag">${s.n_segments} segment${s.n_segments > 1 ? 's' : ''}</span>
      ${parsed.skipped ? `<span class="tag b">${parsed.skipped} skipped (no valid position)</span>` : ''}</div>
    <div class="seg unitsel" role="group" aria-label="Speed unit">${Object.entries(UNITS).map(([k, u]) =>
      `<button type="button" data-unit="${k}" class="${k === unit ? 'on' : ''}" aria-pressed="${k === unit}">${u.label}</button>`).join('')}</div>
    <div class="tiles">
      ${tile(dist(s.distance_m), 'distance')}
      ${hasTime ? tile(dur(s.moving_time_s), `moving time (of ${dur(s.duration_s)})`) : tile('–', 'no timestamps')}
      ${hasTime ? tile(speed(s.moving_speed_mps, unit), `moving average · ${pace(s.moving_speed_mps)}`) : ''}
      ${hasTime ? tile(speed(s.avg_speed_mps, unit), 'overall average') : ''}
      ${hasTime ? tile(speed(s.max_speed_mps, unit), `max over ≥ ${s.settings.speed_window_s} s`) : ''}
      ${hasEle ? tile(`+${num(s.ele_gain_m)} m`, `climb · −${num(s.ele_loss_m)} m descent`) : ''}
      ${hasEle ? tile(`${num(s.ele_min_m)}–${num(s.ele_max_m)} m`, 'elevation range') : ''}
      ${hasHr ? tile(`${num(s.hr_avg)} bpm`, `average heart rate · max ${num(s.hr_max)}`) : ''}
    </div>
    <p class="small mute">Start: ${when(s.start_time)}. Times without a zone are read as UTC.</p>
    <div class="bar"><button class="alt" id="pycheck" type="button" ${isStatic() ? 'hidden' : ''}>Check with the Python modules</button>
      <span class="small" id="pyres" aria-live="polite"></span></div>
    ${isStatic() ? needsServer('Checking these numbers against the Python modules') : ''}</div>
    <h2>Route</h2><div class="panel" id="map"></div>
    <h2>Elevation</h2><div class="panel" id="profile"></div>
    <h2>Splits</h2><div class="panel">${splitsTable(s.splits, s.settings.split_m)}</div>`;
  const map = mountMap($('#map'), parsed.points);
  elevationProfile($('#profile'), parsed.points, s.cumdist_m, i => map.highlight(i));
  out.querySelectorAll('[data-unit]').forEach(b => b.addEventListener('click', () => { unit = b.dataset.unit; renderResult(); }));
  const py = $('#pycheck');
  if (py) py.addEventListener('click', pythonCheck);
}

function analyse(info) {
  if (!current || current.text === undefined) return;
  const c = settingsFrom(info), key = JSON.stringify(c);
  if (current.key === key) return;   // unchanged (e.g. a change event on blur): keep the page, so a click isn't lost
  current.key = key;
  try {
    current.parsed = parseTrack(current.text, current.name);
    current.stats = computeStats(current.parsed.points, c);
    current.error = null;
  } catch (e) { current.error = e.message; }
  renderResult();
}

function differences(a, b, path = '', out = []) {
  if (typeof b === 'number' && typeof a === 'number') {
    if (!(Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b)))) out.push(path);
  } else if (b && typeof b === 'object') {
    if (!a || typeof a !== 'object') out.push(path);
    else for (const k of new Set([...Object.keys(a), ...Object.keys(b)])) differences(a[k], b[k], path ? `${path}.${k}` : k, out);
  } else if (a !== b) out.push(path);
  return out;
}

async function pythonCheck() {
  const res = $('#pyres'), btn = $('#pycheck'), s = current.stats.settings;
  const fd = new FormData();
  fd.append('file', new Blob([current.text]), current.name);
  const q = `moving_speed=${s.moving_speed_mps}&speed_window=${s.speed_window_s}&elevation_threshold=${s.elevation_threshold_m}&split_m=${s.split_m}`;
  btn.disabled = true;
  try {
    const py = await api('/analyze?' + q, { body: fd });
    const diff = differences({ parsed: current.parsed, stats: current.stats }, py);
    res.innerHTML = diff.length
      ? `<span class="err">Python differs in ${diff.length} value(s): ${esc(diff.slice(0, 5).join(', '))} (please report this).</span>`
      : `<span class="ok">✓ Python gives the same ${current.parsed.points.length} points and statistics (within 1e-9).</span>`;
  } catch (e) { res.innerHTML = `<span class="err">${esc(e.message)}</span>`; }
  btn.disabled = false;
}

async function openFile(f, info) {
  current = { name: f.name, text: f.text !== undefined ? f.text : await f.file.text() };
  analyse(info);
}

route('track', async (args, query) => {
  const info = await api('/info');
  current = null;
  app().innerHTML = `<p class="kicker">Analyse a track</p><h1>Open a recorded track</h1>
    <p class="lead">GPX, TCX, CSV (with latitude/longitude columns, e.g. this tracker's own log) or a raw NMEA log.
      Distance, moving time, speed, elevation, heart rate and splits are computed straight away.</p>
    <p class="privacy"><b>Your file stays on this device.</b> It is read by JavaScript in this tab and never
      uploaded${isStatic() ? ' (this site has no server)' : ' (the optional Python check sends it only to the app running on this computer)'}.</p>
    <label class="drop" id="drop" for="pick"><b>Drop a track file here</b><span class="small mute">or choose one (.gpx, .tcx, .csv, .nmea, .txt)</span>
      <input id="pick" type="file" accept=".gpx,.tcx,.csv,.nmea,.txt,.log"></label>
    <div class="bar demos"><span class="small mute">Synthetic demos:</span><span id="demos" class="small mute">loading…</span></div>
    <p class="small mute">The demos are one invented route: a figure-of-eight in the open sea at 0° N, 0° E with a made-up
      hill, heart rate and a one-minute stop (written by <code>scripts/make_demo_tracks.py</code>). No real journey is published here.</p>
    <details class="panel"><summary>Settings</summary>
      <div class="settings">
        <div><label for="set-move">Moving if faster than (km/h)</label><input id="set-move" type="number" min="0" step="0.1" value="${(info.settings.moving_speed_mps * 3.6).toFixed(1)}"></div>
        <div><label for="set-window">Max speed over at least (s)</label><input id="set-window" type="number" min="1" step="1" value="${info.settings.speed_window_s}"></div>
        <div><label for="set-ele">Elevation change counted from (m)</label><input id="set-ele" type="number" min="0.1" step="0.5" value="${info.settings.elevation_threshold_m}"></div>
        <div><label for="set-split">Splits</label><select id="set-split"><option value="1000">per kilometre</option><option value="1609.344">per mile</option></select></div>
      </div><p class="small mute">Definitions are on the <a href="#/about">About</a> page.</p></details>
    <div id="result" aria-live="polite"></div>`;

  const drop = $('#drop'), pick = $('#pick');
  pick.addEventListener('change', () => { if (pick.files[0]) openFile({ name: pick.files[0].name, file: pick.files[0] }, info); pick.value = ''; });
  ['dragenter', 'dragover'].forEach(ev => drop.addEventListener(ev, e => { e.preventDefault(); drop.classList.add('over'); }));
  ['dragleave', 'drop'].forEach(ev => drop.addEventListener(ev, () => drop.classList.remove('over')));
  drop.addEventListener('drop', e => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) openFile({ name: f.name, file: f }, info); });
  for (const id of ['#set-move', '#set-window', '#set-ele', '#set-split']) $(id).addEventListener('change', () => analyse(info));

  const demos = await api('/demo');
  $('#demos').innerHTML = demos.map(d => `<button type="button" class="alt" data-demo="${esc(d.name)}" title="${esc(d.label)}">${esc(d.name.split('.').pop().toUpperCase())}</button>`).join('');
  $('#demos').classList.remove('mute');
  $('#demos').addEventListener('click', e => {
    const b = e.target.closest('[data-demo]');
    if (b) openFile(demos.find(d => d.name === b.dataset.demo), info);
  });
  if (query.demo) { const d = demos.find(x => x.name === query.demo); if (d) await openFile(d, info); }
});
