// Route map: the track drawn in Web Mercator (geo.js, the desktop map's projection) as SVG, with pan (drag, arrow
// keys), zoom (buttons, wheel, +/-), a scale bar sized like the desktop one, and optional OpenStreetMap tiles.
// Tiles are off by default: when on, the browser requests the tiles for the area shown from tile.openstreetmap.org.
import { latlonToTile, tileToLatlon, pan, niceScale, TILE_SIZE } from './geo.js';

const W = 800, H = 480, MIN_Z = 1, MAX_Z = 19;
const OSM = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';

function fitView(points) {
  let lat0 = Infinity, lat1 = -Infinity, lon0 = Infinity, lon1 = -Infinity;
  for (const p of points) { lat0 = Math.min(lat0, p.lat); lat1 = Math.max(lat1, p.lat); lon0 = Math.min(lon0, p.lon); lon1 = Math.max(lon1, p.lon); }
  for (let z = 18; z >= MIN_Z; z--) {
    const [x0, y1] = latlonToTile(lat0, lon0, z), [x1, y0] = latlonToTile(lat1, lon1, z);
    if ((x1 - x0) * TILE_SIZE <= W * 0.86 && (y1 - y0) * TILE_SIZE <= H * 0.86 || z === MIN_Z) {
      const [lat, lon] = tileToLatlon((x0 + x1) / 2, (y0 + y1) / 2, z);
      return { lat, lon, zoom: z };
    }
  }
  return { lat: 0, lon: 0, zoom: MIN_Z };
}

// Screen-space path data per segment, dropping points closer than 0.6 px to the last kept one.
function pathData(points, toScreen) {
  let d = '', last = null, seg = null;
  for (const p of points) {
    const [x, y] = toScreen(p.lat, p.lon);
    if (p.seg !== seg) { d += `M${x.toFixed(1)} ${y.toFixed(1)}`; seg = p.seg; last = [x, y]; continue; }
    if (Math.abs(x - last[0]) + Math.abs(y - last[1]) < 0.6) continue;
    d += `L${x.toFixed(1)} ${y.toFixed(1)}`; last = [x, y];
  }
  return d;
}

export function mountMap(host, points) {
  let view = fitView(points), tiles = false, marker = null;
  host.innerHTML = `<div class="mapbar">
      <button type="button" class="alt" data-z="1" aria-label="Zoom in">+</button>
      <button type="button" class="alt" data-z="-1" aria-label="Zoom out">−</button>
      <button type="button" class="alt" data-fit>Fit route</button>
      <label class="check"><input type="checkbox" data-tiles> OpenStreetMap background</label>
      <span class="small mute" data-status></span></div>
    <svg class="map" viewBox="0 0 ${W} ${H}" tabindex="0" role="group"
      aria-label="Map of the track. Drag or use the arrow keys to pan, + and − to zoom."></svg>
    <p class="small mute" data-note>No map tiles are loaded: the background is a plain grid. Switching the
      OpenStreetMap background on requests map images for the area shown (not your file) from tile.openstreetmap.org.</p>`;
  const svg = host.querySelector('svg'), status = host.querySelector('[data-status]');

  function render() {
    const z = view.zoom, [cx, cy] = latlonToTile(view.lat, view.lon, z), n = 2 ** z;
    const toScreen = (lat, lon) => { const [x, y] = latlonToTile(lat, lon, z); return [W / 2 + (x - cx) * TILE_SIZE, H / 2 + (y - cy) * TILE_SIZE]; };
    let bg = '';
    if (tiles) {
      const tx0 = Math.floor(cx - W / 2 / TILE_SIZE), tx1 = Math.floor(cx + W / 2 / TILE_SIZE);
      const ty0 = Math.max(0, Math.floor(cy - H / 2 / TILE_SIZE)), ty1 = Math.min(n - 1, Math.floor(cy + H / 2 / TILE_SIZE));
      for (let tx = tx0; tx <= tx1; tx++) for (let ty = ty0; ty <= ty1; ty++) {
        const wx = ((tx % n) + n) % n, url = OSM.replace('{z}', z).replace('{x}', wx).replace('{y}', ty);
        bg += `<image href="${url}" x="${(W / 2 + (tx - cx) * TILE_SIZE).toFixed(1)}" y="${(H / 2 + (ty - cy) * TILE_SIZE).toFixed(1)}" width="256" height="256"/>`;
      }
    } else {
      const step = TILE_SIZE / 4, ox = ((-(cx * TILE_SIZE - W / 2)) % step + step) % step, oy = ((-(cy * TILE_SIZE - H / 2)) % step + step) % step;
      for (let x = ox; x < W; x += step) bg += `<line class="grid" x1="${x.toFixed(1)}" y1="0" x2="${x.toFixed(1)}" y2="${H}"/>`;
      for (let y = oy; y < H; y += step) bg += `<line class="grid" x1="0" y1="${y.toFixed(1)}" x2="${W}" y2="${y.toFixed(1)}"/>`;
    }
    const d = pathData(points, toScreen);
    const [sx, sy] = toScreen(points[0].lat, points[0].lon), last = points[points.length - 1], [ex, ey] = toScreen(last.lat, last.lon);
    const [, width, label] = niceScale(view.lat, z, 100);
    let mk = '';
    if (marker !== null && points[marker]) { const [mx, my] = toScreen(points[marker].lat, points[marker].lon); mk = `<circle class="hover" cx="${mx.toFixed(1)}" cy="${my.toFixed(1)}" r="7"/>`; }
    svg.innerHTML = `<rect width="${W}" height="${H}" class="sea"/>${bg}
      <path class="route-glow" d="${d}"/><path class="route" d="${d}"/>
      <circle class="start" cx="${sx.toFixed(1)}" cy="${sy.toFixed(1)}" r="7"><title>Start</title></circle>
      <circle class="end" cx="${ex.toFixed(1)}" cy="${ey.toFixed(1)}" r="6"><title>End</title></circle>${mk}
      <g class="scale" transform="translate(16 ${H - 18})"><rect x="-6" y="-24" width="${(width + 12).toFixed(1)}" height="34" rx="4"/>
        <path d="M0 -6 V2 H${width.toFixed(1)} V-6"/><text x="${(width / 2).toFixed(1)}" y="-10">${label}</text></g>
      ${tiles ? `<a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener"><text class="attrib" x="${W - 8}" y="${H - 8}">© OpenStreetMap contributors</text></a>` : ''}`;
    status.textContent = `zoom ${z} · centre ${view.lat.toFixed(4)}, ${view.lon.toFixed(4)}`;
  }

  const zoom = dz => { view.zoom = Math.max(MIN_Z, Math.min(MAX_Z, view.zoom + dz)); render(); };
  const panBy = (dx, dy) => { [view.lat, view.lon] = pan(view.lat, view.lon, dx, dy, view.zoom); render(); };
  host.querySelectorAll('[data-z]').forEach(b => b.addEventListener('click', () => zoom(+b.dataset.z)));
  host.querySelector('[data-fit]').addEventListener('click', () => { view = fitView(points); render(); });
  host.querySelector('[data-tiles]').addEventListener('change', e => { tiles = e.target.checked; render(); });

  let drag = null;
  svg.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY }; svg.setPointerCapture(e.pointerId); });
  svg.addEventListener('pointermove', e => {
    if (!drag) return;
    const k = W / svg.getBoundingClientRect().width;
    panBy((e.clientX - drag.x) * k, (e.clientY - drag.y) * k);
    drag = { x: e.clientX, y: e.clientY };
  });
  const endDrag = () => { drag = null; };
  svg.addEventListener('pointerup', endDrag); svg.addEventListener('pointercancel', endDrag);
  svg.addEventListener('wheel', e => { e.preventDefault(); zoom(e.deltaY < 0 ? 1 : -1); }, { passive: false });
  svg.addEventListener('keydown', e => {
    const moves = { ArrowLeft: [60, 0], ArrowRight: [-60, 0], ArrowUp: [0, 60], ArrowDown: [0, -60] };
    if (moves[e.key]) { e.preventDefault(); panBy(...moves[e.key]); }
    else if (e.key === '+' || e.key === '=') zoom(1);
    else if (e.key === '-') zoom(-1);
  });
  render();
  return { highlight(i) { marker = i; render(); } };
}

