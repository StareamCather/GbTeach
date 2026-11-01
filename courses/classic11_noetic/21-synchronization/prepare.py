#!/usr/bin/env python3
"""Build the pure native observer and read-only metadata plugin from installed SDK."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
from config import digest, official_folder, read_world


def main():
    config = read_world()
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    if version != "11.15.1":
        raise RuntimeError("L21 requires the installed Gazebo Classic11 SDK")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo"], text=True, timeout=5).strip()
    folder = official_folder()
    sources = {name: digest(folder/name) for name in ("native_observer.cc", "metadata_plugin.cc")}
    key = hashlib.sha256((json.dumps(sources, sort_keys=True)+version+flags).encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/synchronization"/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    products = {"observer": ("native_observer.cc", cache/"native_observer", []),
                "metadata_library": ("metadata_plugin.cc", cache/"libgazeboteach_l21_metadata.so", ["-shared", "-fPIC"])}
    record = {"world_sha256": digest(Path.cwd()/"world.sdf"), "config": config,
              "source_sha256": sources, "build_sha256": key, "gazebo_version": version}
    for name, (source, destination, extra) in products.items():
        if not destination.is_file():
            temporary = cache/("build-"+uuid.uuid4().hex)
            try:
                subprocess.run(["g++", "-std=c++17", "-O2", "-Wno-deprecated-declarations"]+extra+
                               [str(folder/source), "-o", str(temporary)]+shlex.split(flags)+["-ldl"], check=True, timeout=120)
                os.replace(str(temporary), str(destination))
            finally:
                if temporary.exists():
                    temporary.unlink()
        record[name] = str(destination)
        record[name+"_sha256"] = digest(destination)
    library = Path.cwd()/"build/lib/libgazeboteach_l21_metadata.so"
    library.parent.mkdir(parents=True, exist_ok=True)
    if library.exists() or library.is_symlink():
        library.unlink()
    library.symlink_to(record["metadata_library"])
    output = Path.cwd()/"build/sync_lab.json"
    temporary = output.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(output))
    print(json.dumps({"prepared": True, **record}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
