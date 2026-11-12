"""Web interface for the GPS tracker.

Contents:
- app.py     FastAPI app: front end in static/, JSON API under /api (info, demo tracks, Python analysis of an upload)
- static/    plain ES-module front end (no build step); track_io/track_nmea/track_values/track_stats/geo/xml_lite are
             JavaScript ports of the Python modules, checked by static/track_engine.test.mjs
"""
