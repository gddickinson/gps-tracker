// Entry point: registers lazily loaded pages, sets the footer mode text, draws the background and starts the router.
import { start, lazy, route, app, isStatic, $ } from './core.js';

const PAGES = { 'home.js': ['home'], 'track.js': ['track'], 'about.js': ['about'] };
lazy(Object.fromEntries(Object.entries(PAGES).flatMap(([file, names]) => names.map(n => [n, () => import('./' + file)]))));
route('notfound', () => { app().innerHTML = '<h1>Not found</h1><p><a href="#/">Back to the start</a></p>'; });

$('#mode').textContent = isStatic()
  ? 'static site: tracks are read in your browser and never uploaded'
  : 'local app: browser analysis plus the Python modules on this computer';

// Subtle background: a few slow contour-like rings drifting (drawn once under prefers-reduced-motion).
(function background() {
  const cv = $('#bg'), ctx = cv.getContext('2d');
  const still = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const draw = t => {
    const w = cv.width = innerWidth, h = cv.height = innerHeight;
    ctx.clearRect(0, 0, w, h);
    for (let k = 0; k < 8; k++) {
      ctx.strokeStyle = k % 2 ? 'rgba(47,243,200,.28)' : 'rgba(255,92,168,.22)';
      ctx.lineWidth = 1.1; ctx.beginPath();
      for (let a = 0; a <= 64; a++) {
        const th = a / 64 * Math.PI * 2, r = 60 + k * 46 + 10 * Math.sin(3 * th + k + t / 4000);
        const x = w * 0.78 + r * 1.5 * Math.cos(th), y = h * 0.32 + r * Math.sin(th);
        a ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }
      ctx.stroke();
    }
  };
  if (still) { draw(0); addEventListener('resize', () => draw(0)); return; }
  let last = 0;
  const loop = t => { if (t - last > 80) { draw(t); last = t; } requestAnimationFrame(loop); };
  requestAnimationFrame(loop);
})();

start();
