// Helper for tests/test_js_port.py: reads [[text, filename], ...] as JSON on stdin and prints, for each, the JS
// parse + statistics (or "error"), as JSON.
import { parseTrack } from '../web/static/track_io.js';
import { computeStats } from '../web/static/track_stats.js';

let input = '';
process.stdin.on('data', d => { input += d; });
process.stdin.on('end', () => {
  const out = JSON.parse(input).map(([text, name]) => {
    try { const p = parseTrack(text, name); return { parsed: p, stats: computeStats(p.points) }; } catch (e) { return 'error'; }
  });
  process.stdout.write(JSON.stringify(out));
});
