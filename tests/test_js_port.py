"""Fuzz the JS port against Python: seeded random mutations of the fixture tracks must parse to the same points and
statistics in both (or be rejected by both). Needs node; skipped without it."""
import json
import math
import random
import shutil
import subprocess
from pathlib import Path

import pytest

import track_io
import track_stats

FIX = Path(__file__).resolve().parent / "fixtures"
RUNNER = Path(__file__).resolve().parent / "js_port_runner.mjs"
SOURCES = ["demo_route.gpx", "demo_route.tcx", "demo_log.csv", "demo_log.nmea", "edge_cases.gpx", "edge_cases.csv"]
SNIPPETS = ["", "0", "-", ".", "1e5", "nan", " ", "\n", ",", ";", "\"", "Z", "+02:00", "99", "A", "V", "*", "$GPRMC,",
            "<ele>3</ele>", "</trkseg><trkseg>", "&amp;", "<!-- c -->", "<![CDATA[7]]>"]


def mutate(text, rnd):
    lines = text.splitlines(keepends=True)
    for _ in range(rnd.randint(1, 4)):
        op = rnd.random()
        k = rnd.randrange(len(lines))
        if op < 0.3 and len(lines) > 3:
            del lines[k]
        elif op < 0.5:
            lines.insert(k, lines[rnd.randrange(len(lines))])
        else:
            ln = lines[k]
            i = rnd.randrange(len(ln) + 1)
            j = min(len(ln), i + rnd.randint(0, 3))
            lines[k] = ln[:i] + rnd.choice(SNIPPETS) + ln[j:]
    return "".join(lines)


def py_result(text, name):
    try:
        p = track_io.parse_track(text, name)
        return {"parsed": p, "stats": track_stats.compute_stats(p["points"])}
    except ValueError:
        return "error"


def same(a, b):
    if isinstance(b, float) and isinstance(a, (int, float)):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
    if isinstance(b, dict):
        return isinstance(a, dict) and a.keys() == b.keys() and all(same(a[k], b[k]) for k in b)
    if isinstance(b, list):
        return isinstance(a, list) and len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return a == b


@pytest.mark.skipif(not shutil.which("node"), reason="node not installed")
def test_js_port_matches_python_on_mutated_files():
    rnd = random.Random(2026)
    cases = []
    for k in range(240):
        name = SOURCES[k % len(SOURCES)]
        text = (FIX / name).read_text(encoding="utf-8")
        if name.startswith("demo"):
            text = "".join(text.splitlines(keepends=True)[:60])   # keep the runs quick
        cases.append([mutate(text, rnd), name])
    r = subprocess.run(["node", str(RUNNER)], input=json.dumps(cases), capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    js = json.loads(r.stdout)
    diffs = [i for i, (c, j) in enumerate(zip(cases, js)) if not same(j, py_result(*c))]
    assert not diffs, f"{len(diffs)} differences, first: {cases[diffs[0]][1]} {cases[diffs[0]][0][:400]!r}"
