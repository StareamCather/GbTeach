#!/usr/bin/env python3
"""Build the real Classic force controller and its bounded SDK client offline."""
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
    p = read_parameters(workspace/"parameters.json")
    read_world(workspace/"world.sdf", official/"world.sdf")
    version = subprocess.check_output(["pkg-config", "--modversion", "gazebo"], text=True, timeout=5).strip()
    if version != "11.15.1":
        raise RuntimeError("Transport gate requires the Gazebo Classic 11 SDK")
    flags = subprocess.check_output(["pkg-config", "--cflags", "--libs", "gazebo"], text=True, timeout=5).strip()
    key = hashlib.sha256(b"".join((official/name).read_bytes() for name in ("transport_gate.cc", "transport_client.cc"))+(version+flags).encode()).hexdigest()
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home()/".cache"))/"gazeboteach/transport-gate"/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    outputs = (("transport_gate.cc", "libgazeboteach_transport_gate.so", ["-shared", "-fPIC"]),
               ("transport_client.cc", "transport_client", []))
    for source, name, extra in outputs:
        output = cache/name
        if not output.is_file():
            temporary = cache/("build-"+uuid.uuid4().hex)
            try:
                subprocess.run(["g++", "-std=c++17", "-O2", "-Wno-deprecated-declarations"]+extra+
                               [str(official/source), "-o", str(temporary)]+shlex.split(flags)+["-ldl"], check=True, timeout=60)
                os.replace(str(temporary), str(output))
            finally:
                if temporary.exists():
                    temporary.unlink()
        target = workspace/("build/lib/"+name if name.endswith(".so") else "build/"+name)
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() or target.is_symlink():
            target.unlink()
        target.symlink_to(output)
    metadata = {"config_sha256": digest(workspace/"parameters.json"), "world_sha256": digest(workspace/"world.sdf"),
                "source_sha256": digest(official/"transport_gate.cc"), "client_source_sha256": digest(official/"transport_client.cc"),
                "build_sha256": key, "gazebo_version": version, "parameters": p}
    (workspace/"build/transport_gate.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    values = [metadata[key] for key in ("config_sha256", "world_sha256", "source_sha256", "client_source_sha256")]
    values += ["%s %.17g" % (key, value) for key, value in sorted(p.items())]
    (workspace/"build/transport_gate.conf").write_text("\n".join(values)+"\n", encoding="ascii")
    print("Classic Transport force controller and bounded SDK client ready:", cache, flush=True)


if __name__ == "__main__":
    build()
