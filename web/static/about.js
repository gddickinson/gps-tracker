// About page: what the project is, the statistic definitions (values from the API), the desktop app, running the web
// app, how the browser port was checked, privacy and credits.
import { route, app, api, esc } from './core.js';

route('about', async () => {
  const info = await api('/info'), st = info.settings, cols = info.csv_columns;
  app().innerHTML = `<p class="kicker">About</p><h1>GPS Tracker</h1>
    <p class="lead">${esc(info.description)}</p>
    <h2>The desktop app</h2>
    <p>The Python app (PyQt6) shows live data from a serial GPS receiver (NMEA RMC and GGA sentences, read with pynmea2)
      or an approximate position from IP geolocation: time, position, speed in knots, course, fix quality, satellites,
      altitude, HDOP and the track distance. It logs to CSV, and its map window draws the track over cached tiles from
      ${info.tile_servers.map(esc).join(', ')}, with a scale bar, dragging and zoom. Run it with <code>pip install -r requirements.txt</code>
      and <code>python main.py</code>.</p>
    <h2>This web version</h2>
    <p>The web pages work with tracks you have already recorded. The file is parsed and analysed in your browser by
      JavaScript ports of the repository's Python modules <code>track_io.py</code>, <code>track_nmea.py</code>,
      <code>track_stats.py</code> and <code>geo.py</code> (the map projection and scale bar of the desktop map).</p>
    <ul>
      <li><b>GPX</b>: track segments (trkseg) and routes (rte); elevation, time and heart rate from extensions.</li>
      <li><b>TCX</b>: each Track is a segment; position, altitude, time and heart rate per Trackpoint.</li>
      <li><b>CSV</b>: comma, semicolon or tab separated, header names in any case: latitude (${cols.lat.map(esc).join(', ')}),
        longitude (${cols.lon.map(esc).join(', ')}), elevation (${cols.ele.map(esc).join(', ')}), time (${cols.time.map(esc).join(', ')})
        and heart rate (${cols.hr.map(esc).join(', ')}). The desktop app's own log has these columns.</li>
      <li><b>NMEA</b>: one point per RMC sentence with a valid fix, its altitude from the latest GGA; any talker (GP, GN,
        GL…); sentences with a wrong checksum are skipped.</li>
    </ul>
    <h2>How the statistics are defined</h2>
    <ul>
      <li><b>Distance</b>: sum of great-circle (haversine, Earth radius 6371 km) distances between consecutive points; nothing
        is counted across a gap between segments.</li>
      <li><b>Duration</b>: earliest to latest timestamp. <b>Moving time</b>: the time steps faster than ${st.moving_speed_mps} m/s
        (${(st.moving_speed_mps * 3.6).toFixed(1)} km/h); moving average = distance in those steps / moving time.</li>
      <li><b>Max speed</b>: the fastest average over any stretch of at least ${st.speed_window_s} s, which smooths GPS jitter.</li>
      <li><b>Climb and descent</b>: a change is counted once the elevation has moved ${st.elevation_threshold_m} m from the
        last turning point, so noise of a metre or two is not added up.</li>
      <li><b>Splits</b>: the time of each whole ${st.split_m / 1000} km (crossing times interpolated) and of the rest.</li>
      <li>Times without a time zone are read as UTC; only differences matter for the statistics.</li>
    </ul>
    <p>All four settings can be changed on the track page.</p>
    <h2>Checked against the Python</h2>
    <p><code>web/static/track_engine.test.mjs</code> (node) compares the JavaScript with values written by the Python modules
      for every demo file, hand-written edge cases (namespaces, entities, CDATA, missing values, semicolons, BOM, time
      zones, bad checksums, lost fixes) and tables of numbers, times and map maths. <code>tests/test_js_port.py</code> runs
      both on hundreds of randomly damaged files and requires the same points and statistics, or the same rejection.
      In the local app, "Check with the Python modules" repeats the comparison for your own file.</p>
    <h2>Privacy</h2>
    <p>Files are never uploaded by the static site. The map is drawn without tiles by default; the optional OpenStreetMap
      background loads tiles for the area shown from tile.openstreetmap.org (© OpenStreetMap contributors). The demo
      tracks are synthetic, in the open sea at 0° N, 0° E.</p>
    <h2>Run the web app locally</h2>
    <pre>pip install -r requirements-web.txt
python run_web.py        # http://127.0.0.1:8161</pre>
    <h2>Credits</h2>
    <p>By George Dickinson, built with AI assistance from Claude (Anthropic). Map data and tiles in the desktop app and
      the optional background: OpenStreetMap and contributors. NMEA parsing: pynmea2. MIT licence.</p>`;
});
