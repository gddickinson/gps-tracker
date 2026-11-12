"""Start the local web app: python run_web.py  ->  http://127.0.0.1:8161"""
import sys
from pathlib import Path

import uvicorn

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

if __name__ == "__main__":
    print("GPS Tracker web app: http://127.0.0.1:8161")
    uvicorn.run("web.app:app", host="127.0.0.1", port=8161)
