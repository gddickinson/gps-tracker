// Static-site data layer: answers api() calls without the Python server.
// GET /info and /demo come from the JSON exported by scripts/build_static.py. Track analysis on the static site never
// calls api(): the track page runs the JS port (track_io.js, track_stats.js) in the browser; the Python check needs
// the local app.
const files = new Map();
function file(rel) {
  if (!files.has(rel)) files.set(rel, fetch('api/' + rel + '.json').then(r => { if (!r.ok) throw new Error(`${r.status} ${rel}`); return r.json(); }));
  return files.get(rel);
}

export async function staticApi(path, body) {
  const clean = path.split('?')[0].replace(/^\/+/, '');
  if (body) throw new Error(`${clean} runs Python and needs the local app (python run_web.py).`);
  return file(clean);
}
