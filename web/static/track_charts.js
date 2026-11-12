// Charts for the track page: elevation profile (SVG, hover shows distance/elevation and moves the map marker) and
// the per-split bar list.
import { dist, dur, pace } from './format.js';

const W = 800, H = 200, PAD = { l: 46, r: 12, t: 12, b: 26 };

function niceTicks(lo, hi, count = 4) {
  const span = hi - lo || 1, raw = span / count, p = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 5, 10].map(s => s * p).find(s => s >= raw);
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toPrecision(12));
  return out;
}

export function elevationProfile(host, points, cumdist, onHover) {
  const idx = points.map((p, i) => i).filter(i => points[i].ele !== null);
  if (idx.length < 2) { host.innerHTML = '<p class="small mute">No elevation in this file.</p>'; return; }
  const total = cumdist[cumdist.length - 1] || 1;
  let lo = Infinity, hi = -Infinity;
  for (const i of idx) { lo = Math.min(lo, points[i].ele); hi = Math.max(hi, points[i].ele); }
  if (hi - lo < 10) { const mid = (hi + lo) / 2; lo = mid - 5; hi = mid + 5; }
  const X = d => PAD.l + d / total * (W - PAD.l - PAD.r), Y = e => H - PAD.b - (e - lo) / (hi - lo) * (H - PAD.t - PAD.b);
  const stride = Math.max(1, Math.floor(idx.length / 1600));
  let line = '';
  idx.forEach((i, k) => { if (k % stride === 0 || k === idx.length - 1) line += `${line ? 'L' : 'M'}${X(cumdist[i]).toFixed(1)} ${Y(points[i].ele).toFixed(1)}`; });
  const area = `${line}L${X(cumdist[idx[idx.length - 1]]).toFixed(1)} ${H - PAD.b}L${X(cumdist[idx[0]]).toFixed(1)} ${H - PAD.b}Z`;
  const yt = niceTicks(lo, hi), xt = niceTicks(0, total / 1000, 6).filter(v => v * 1000 <= total);
  host.innerHTML = `<svg class="profile" viewBox="0 0 ${W} ${H}" role="img" aria-label="Elevation profile from ${Math.round(lo)} to ${Math.round(hi)} m over ${dist(total)}">
    ${yt.map(v => `<line class="grid" x1="${PAD.l}" x2="${W - PAD.r}" y1="${Y(v).toFixed(1)}" y2="${Y(v).toFixed(1)}"/><text class="ax" x="${PAD.l - 6}" y="${(Y(v) + 4).toFixed(1)}" text-anchor="end">${v} m</text>`).join('')}
    ${xt.map(v => `<text class="ax" x="${X(v * 1000).toFixed(1)}" y="${H - 8}" text-anchor="middle">${v} km</text>`).join('')}
    <path class="area" d="${area}"/><path class="line" d="${line}"/>
    <g class="cursor" visibility="hidden"><line y1="${PAD.t}" y2="${H - PAD.b}"/><circle r="4"/><text y="${PAD.t + 10}"></text></g></svg>`;
  const svg = host.querySelector('svg'), cur = svg.querySelector('.cursor');
  svg.addEventListener('pointermove', e => {
    const r = svg.getBoundingClientRect(), x = (e.clientX - r.left) * W / r.width;
    const d = (x - PAD.l) / (W - PAD.l - PAD.r) * total;
    let best = idx[0];
    for (const i of idx) { if (Math.abs(cumdist[i] - d) < Math.abs(cumdist[best] - d)) best = i; }
    const px = X(cumdist[best]), py = Y(points[best].ele);
    cur.setAttribute('visibility', 'visible');
    cur.querySelector('line').setAttribute('x1', px); cur.querySelector('line').setAttribute('x2', px);
    cur.querySelector('circle').setAttribute('cx', px); cur.querySelector('circle').setAttribute('cy', py);
    const t = cur.querySelector('text');
    t.setAttribute('x', Math.min(px + 6, W - 150)); t.textContent = `${dist(cumdist[best])} · ${points[best].ele.toFixed(1)} m`;
    onHover(best);
  });
  svg.addEventListener('pointerleave', () => { cur.setAttribute('visibility', 'hidden'); onHover(null); });
}

export function splitsTable(splits, splitM) {
  if (!splits.length) return '<p class="small mute">Splits need timestamps.</p>';
  const speeds = splits.map(s => (s.time_s > 0 ? s.distance_m / s.time_s : 0)), fast = Math.max(...speeds) || 1;
  const unit = splitM === 1000 ? 'km' : splitM === 1609.344 ? 'mile' : `${splitM} m`;
  return `<div class="scroll"><table class="splits"><thead><tr><th>${unit}</th><th>distance</th><th>time</th><th>pace</th><th class="barcol">speed</th></tr></thead><tbody>
    ${splits.map((s, k) => `<tr><td>${k + 1}</td><td>${dist(s.distance_m)}</td><td>${dur(s.time_s)}</td><td>${pace(speeds[k])}</td>
      <td class="barcol"><span class="bar" style="width:${(speeds[k] / fast * 100).toFixed(1)}%"></span></td></tr>`).join('')}
    </tbody></table></div>`;
}
