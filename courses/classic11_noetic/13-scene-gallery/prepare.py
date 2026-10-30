#!/usr/bin/env python3
"""Validate inputs and cache the two local Classic SDK helpers, fully offline."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
from config import digest, read_world, resources


def build():
    workspace = Path.cwd()
    official = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))
    parameters = read_world(workspace/"world.sdf", official/"world.sdf")
    assets = resources(workspace)
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    if not version.startswith("11."):
        raise RuntimeError("Scene gallery requires the Gazebo Classic 11 SDK")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo"], text=True, timeout=5).strip()
    key = hashlib.sha256(b"".join((official/n).read_bytes() for n in ("gallery.cc", "gallery_camera.cc"))+(version+flags).encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/scene-gallery"/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    for source, name, extra in (("gallery.cc", "libgazeboteach_gallery.so", ["-shared", "-fPIC"]), ("gallery_camera.cc", "gallery_camera", [])):
        output = cache/name
        if not output.is_file():
            temporary = cache/("build-"+uuid.uuid4().hex)
            try:
                subprocess.run(["g++", "-std=c++17", "-O2", "-Wno-deprecated-declarations"]+extra+[str(official/source), "-o", str(temporary)]+shlex.split(flags), check=True, timeout=70)
                os.replace(str(temporary), str(output))
            finally:
                if temporary.exists():
                    temporary.unlink()
        target = workspace/("build/lib/"+name if name.endswith(".so") else "build/"+name)
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() or target.is_symlink():
            target.unlink()
        target.symlink_to(output)
    metadata = {"world_sha256": digest(workspace/"world.sdf"), "source_sha256": digest(official/"gallery.cc"),
                "client_source_sha256": digest(official/"gallery_camera.cc"), "png_sha256": assets["heightmap"]["sha256"],
                "build_sha256": key, "gazebo_version": version, "assets": assets, "parameters": parameters}
    (workspace/"build/gallery.json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    print("Offline scene gallery SDK helpers ready:", cache, flush=True)


if __name__ == "__main__":
    build()
