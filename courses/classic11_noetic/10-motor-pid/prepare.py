#!/usr/bin/env python3
"""Validate the current controls and build only the bounded observation helper."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
from config import digest, read_parameters


def build():
    workspace = Path.cwd()
    official = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))
    parameters = read_parameters(workspace/"parameters.json")
    if digest(workspace/"world.sdf") != digest(official/"world.sdf"):
        raise RuntimeError("This PID measurement uses fixed world.sdf; edit parameters.json, or restore the world template")
    source = official/"motor_lab.cc"
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    if not version.startswith("11."):
        raise RuntimeError("Motor laboratory requires the Gazebo Classic 11 SDK")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo"], text=True, timeout=5).strip()
    key = hashlib.sha256(source.read_bytes()+(version+flags+" -lgazebo_ode").encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/motor-lab"/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    library = cache/"libgazeboteach_motor_lab.so"
    if not library.is_file():
        temporary = cache/("build-"+uuid.uuid4().hex+".so")
        try:
            subprocess.run(["g++", "-shared", "-fPIC", "-std=c++17", "-O2", "-Wno-deprecated-declarations",
                            str(source), "-o", str(temporary)]+shlex.split(flags)+["-lgazebo_ode"], check=True, timeout=60)
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
                "source_sha256": digest(source), "build_sha256": key, "library": str(library), "gazebo_version": version,
                "parameters": parameters, "extra_sdk_libraries": ["gazebo_ode"]}
    (workspace/"build/motor_lab.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    lines = [metadata["config_sha256"], metadata["world_sha256"], metadata["source_sha256"]]
    lines += ["%s %.17g" % (name, value) for name, value in sorted(parameters.items())]
    (workspace/"build/motor_lab.conf").write_text("\n".join(lines)+"\n", encoding="ascii")
    print("Native JointController observation/load helper ready:", library, flush=True)


if __name__ == "__main__":
    build()
