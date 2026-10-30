#!/usr/bin/env python3
import os
from pathlib import Path
import sys

official = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))
sys.path.insert(0, str(official.parent/"_control_support"))
from build import build

if __name__ == "__main__":
    build()
