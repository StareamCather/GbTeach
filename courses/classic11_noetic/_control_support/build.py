"""Build the course-owned bridge against the installed Gazebo/Python SDK."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid


def build():
    source = Path(__file__).with_name("python_control.cc")
    def output(*args):
        return subprocess.check_output(args, text=True, timeout=5).strip()
    version = output("pkg-config", "--modversion", "gazebo")
    if not version.startswith("11."):
        raise RuntimeError("This bridge requires the installed Gazebo Classic 11 SDK")
    flags = output("pkg-config", "--cflags", "--libs", "gazebo")
    includes = output("python3-config", "--includes")
    links = output("python3-config", "--embed", "--ldflags")
    subprocess.run(["python3", "-c", "import numpy; print('NumPy import OK:', numpy.__version__)"],
                   check=True, timeout=5)
    digest = hashlib.sha256(source.read_bytes() + (version+flags+includes+links).encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/python-control"/digest[:24]
    cache.mkdir(parents=True, exist_ok=True)
    library = cache/"libgazeboteach_python_control.so"
    if not library.is_file():
        temporary = cache/("build-"+uuid.uuid4().hex+".so")
        try:
            subprocess.run(["g++", "-shared", "-fPIC", "-std=c++17", "-O2", "-Wno-deprecated-declarations",
                            str(source), "-o", str(temporary)]+shlex.split(flags+" "+includes+" "+links),
                           check=True, timeout=65)
            os.replace(str(temporary), str(library))
        finally:
            if temporary.exists():
                temporary.unlink()
    destination = Path.cwd()/"build/lib"
    destination.mkdir(parents=True, exist_ok=True)
    link = destination/library.name
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(library)
    metadata = {"library": str(library), "build_sha256": digest, "gazebo_version": version,
                "python_embed_flags": links, "control_period_s": .01, "physics_step_s": .001}
    (destination.parent/"control_bridge.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print("Actual-state Python control bridge ready:", library, flush=True)
    return metadata
