"""Shared test setup: repo root on sys.path, Qt offscreen, and no network or tile cache writes from the map widget."""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.fixture
def qapp():
    """A QApplication for widget tests (offscreen)."""
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def map_widget(qapp, monkeypatch, tmp_path):
    """MapWidget whose TileManager uses a temp cache and never touches the network."""
    import mapping
    monkeypatch.setattr(mapping.TileManager, "_check_internet", lambda self: True)
    real_init = mapping.TileManager.__init__
    monkeypatch.setattr(mapping.TileManager, "__init__",
                        lambda self, cache_dir=None: real_init(self, str(tmp_path / "cache")))
    return mapping.MapWidget()
