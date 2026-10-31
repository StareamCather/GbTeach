#!/usr/bin/env python3
"""Build an adapter for five actual native protobuf sensor topics; never generate sensor values."""
import json
import os
from pathlib import Path
import hashlib
import shlex
import subprocess
import uuid
from config import digest, official_folder, read_world


def main():
    world = Path.cwd()/"world.sdf"
    config = read_world(world)
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    if not version.startswith("11."):
        raise RuntimeError("L20 native sensor stand adapter requires the Gazebo Classic11 SDK")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo", "roscpp", "std_msgs"], text=True, timeout=5).strip()
    source = official_folder()/"sensor_bridge.cc"
    key = hashlib.sha256(source.read_bytes()+(version+flags).encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/sensor-stand"/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    library = cache/"libgazeboteach_l20_sensor_bridge.so"
    if not library.is_file():
        temporary = cache/("build-"+uuid.uuid4().hex+".so")
        try:
            subprocess.run(["g++", "-shared", "-fPIC", "-std=c++17", "-O2", "-Wno-deprecated-declarations",
                            str(source), "-o", str(temporary)]+shlex.split(flags), check=True, timeout=120)
            os.replace(str(temporary), str(library))
        finally:
            if temporary.exists():
                temporary.unlink()
    destination = Path.cwd()/"build/lib"
    destination.mkdir(parents=True, exist_ok=True)
    link = destination/library.name
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(library)
    output = Path.cwd()/"build/stand_lab.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    data = {"world_sha256": digest(world), "config": config,
            "native_bridge_source_sha256": digest(source), "native_bridge_library_sha256": digest(library),
            "native_bridge_library": str(library), "build_sha256": key, "gazebo_version": version}
    temporary = output.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(output))
    print(json.dumps({"prepared": True, **data}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
