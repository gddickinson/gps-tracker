// Home page: hero, privacy note, counts from the API and links to the pages.
import { route, app, api, isStatic, esc } from './core.js';

route('home', async () => {
  const info = await api('/info');
  app().innerHTML = `<section class="hero"><p class="kicker">GPS tracks · statistics · map</p>
    <h1 class="glow">GPS Tracker</h1>
    <p class="lead">${esc(info.description)} This web version opens tracks you have already recorded and computes the
      statistics: distance, moving time, speed, elevation, heart rate and kilometre splits, with the route on a map.</p>
    <a class="btn" href="#/track">Open a track</a> <a class="btn alt" href="#/track?demo=demo_route.gpx">Try the demo route</a></section>
    <p class="privacy"><b>Your files stay on this device.</b> ${isStatic()
      ? 'Tracks are read and analysed by JavaScript in this browser tab. Nothing is uploaded: this site has no server.'
      : 'Tracks are analysed in this browser tab; the optional Python check sends a file only to the app running on this computer.'}
      The map is drawn without map tiles unless you switch them on.</p>
    <div class="tiles">
      <div class="tile"><b>${info.formats.length}</b><span>file formats: ${info.formats.map(esc).join(', ').toUpperCase()}</span></div>
      <div class="tile"><b>0</b><span>bytes uploaded</span></div>
      <div class="tile"><b>${info.demos}</b><span>synthetic demo tracks</span></div>
      <div class="tile"><b>${info.tile_servers.length}</b><span>map sources in the desktop app</span></div></div>
    <div class="grid">
      <a class="card" href="#/track"><b>Analyse a track</b><small>GPX · TCX · CSV · NMEA</small><span>Drop a file from a
        watch, phone app or this tracker's own CSV log or raw NMEA output. See the route, elevation profile and splits.</span></a>
      <a class="card" href="#/track?demo=demo_log.nmea"><b>Read an NMEA log</b><small>raw receiver output</small><span>RMC
        and GGA sentences, read the same way as the desktop app reads a serial GPS (pynmea2 rules, checksums checked).</span></a>
      <a class="card" href="#/about"><b>About</b><small>definitions · desktop app · run it locally</small><span>How each
        statistic is defined, the desktop tracker (serial GPS, IP location, tile map, CSV logging) and how the browser
        version was checked against the Python.</span></a>
    </div>`;
});
