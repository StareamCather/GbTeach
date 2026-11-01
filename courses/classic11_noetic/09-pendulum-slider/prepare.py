#!/usr/bin/env python3
"""Compile the native reader / bounded-step sampler into a keyed user cache."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid


def build():
    workspace = Path.cwd().resolve()
    source = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent)) / "rig_sampler.cc"
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=4).strip()
    if version != "11.15.1":
        raise RuntimeError("Native IO lifetime guard requires Gazebo Classic SDK 11.15.1")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo"], text=True, timeout=4).strip()
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    digest = hashlib.sha256((source_hash+version+flags).encode()).hexdigest()[:20]
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "gazeboteach/rig-sampler" / digest
    cache.mkdir(parents=True, exist_ok=True)
    binary = cache / "rig_sampler"
    if not binary.is_file():
        temporary = cache / ("rig_sampler-" + uuid.uuid4().hex)
        try:
            subprocess.run(["g++", "-std=c++17", "-Wno-deprecated-declarations", str(source), "-o", str(temporary)]+
                           shlex.split(flags)+["-ldl"], check=True, timeout=32)
            temporary.chmod(0o755)
            os.replace(str(temporary), str(binary))
        finally:
            if temporary.exists():
                temporary.unlink()
    build_dir = workspace / "build"
    build_dir.mkdir(exist_ok=True)
    metadata = {"binary": str(binary), "source_sha256": source_hash, "gazebo_version": version,
                "purpose": "passive_rig_native_observation_and_bounded_world_steps", "request_timeout_s": 1.5}
    (build_dir / "rig_sampler.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print("原生关节读取与有界步进采样器就绪：" + str(binary), flush=True)
    return metadata


if __name__ == "__main__":
    build()
