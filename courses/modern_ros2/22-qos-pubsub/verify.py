import os
from pathlib import Path
import sys
root = Path(os.environ.get("GAZEBOTEACH_SOURCE_ROOT", str(Path(__file__).resolve().parents[3])))
sys.path.insert(0, str(root / "courses/modern_ros2/_support"))
sys.path.insert(0, str(Path.cwd()))
from lab import experiment
from common import result
if __name__ == "__main__":
    result(experiment())
