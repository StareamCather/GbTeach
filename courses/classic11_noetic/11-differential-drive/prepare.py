#!/usr/bin/env python3
"""Build the observer; the personal SDF is validated and never overwritten."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
from config import digest, read_parameters, read_world


def build():
    workspace = Path.cwd()
    official = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))
    parameters = read_parameters(workspace/"parameters.json")
    world = read_world(workspace/"world.sdf", official/"world.sdf")
    source = official/"drive_lab.cc"
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    if not version.startswith("11."):
        raise RuntimeError("Differential drive requires the Gazebo Classic 11 SDK")
    stock = Path("/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libDiffDrivePlugin.so")
    if not stock.is_file():
        raise RuntimeError("Installed native libDiffDrivePlugin.so is missing")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo"], text=True, timeout=5).strip()
    key = hashlib.sha256(source.read_bytes()+(version+flags).encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/drive-lab"/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    library = cache/"libgazeboteach_drive_lab.so"
    if not library.is_file():
        temporary = cache/("build-"+uuid.uuid4().hex+".so")
        try:
            subprocess.run(["g++", "-shared", "-fPIC", "-std=c++17", "-O2", "-Wno-deprecated-declarations",
                            str(source), "-o", str(temporary)]+shlex.split(flags), check=True, timeout=60)
            os.replace(str(temporary), str(library))
        finally:
            if temporary.exists():
                temporary.unlink()
    destination = workspace/"build/lib"
    destination.mkdir(parents=True, exist_ok=True)
    link = destination/library.name
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(library)
    metadata = {"config_sha256": digest(workspace/"parameters.json"), "world_sha256": digest(workspace/"world.sdf"),
                "source_sha256": digest(source), "stock_plugin_sha256": digest(stock), "build_sha256": key,
                "library": str(library), "gazebo_version": version, "parameters": parameters, "world": world}
    (workspace/"build/drive_lab.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    lines = [metadata[key] for key in ("config_sha256", "world_sha256", "source_sha256", "stock_plugin_sha256")]
    lines += ["surface_mode "+world["surface_mode"]]
    lines += ["%s %.17g" % (name, value) for name, value in sorted(parameters.items())]
    (workspace/"build/drive_lab.conf").write_text("\n".join(lines)+"\n", encoding="ascii")
    print("Native DiffDrivePlugin publisher and read-only observer ready:", library, flush=True)


if __name__ == "__main__":
    build()
