# Archive

`gps-tracker.py` is the original single-file version of the app. It is **superseded** by `main.py` and the modules
(`providers.py`, `mapping.py`, `gui.py`, `geo.py`, ...) and is kept only for reference. Nothing imports it.

It still has **known bugs** that are fixed in the modules: map dragging moves north/south the wrong way, the scale bar
length does not match its label, the serial GPS reads one NMEA line per second (growing lag), never logs GGA
altitude/satellites, closes the port on a corrupt line and ignores `$GN` receivers, zero readings are not shown, and
the Distance field is never filled in. Do not run it; use `python main.py`.
