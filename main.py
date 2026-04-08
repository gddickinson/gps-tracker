#!/usr/bin/env python3
"""
GPS Tracker - Entry point

Launches the GPS tracking application with PyQt6 GUI.
"""

import sys
import logging

from PyQt6.QtWidgets import QApplication
from gui import GPSWindow


def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)
    window = GPSWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
