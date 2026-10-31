#!/usr/bin/env python3
"""Manage only this session's actual RViz PID; children retain the caller's group/token."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from config import PACKAGE, prepared


def identity(pid):
    try:
        base = Path("/proc")/str(pid)
        data = (base/"stat").read_text().split(")", 1)[1].split()
        environment = (base/"environ").read_bytes().split(b"\0")
        token = ("GAZEBOTEACH_SESSION_TOKEN="+os.environ["GAZEBOTEACH_SESSION_TOKEN"]).encode()
        command = (base/"cmdline").read_bytes().split(b"\0")
        if base.stat().st_uid != os.getuid() or token not in environment or not command or Path(os.fsdecode(command[0])).name != "rviz":
            return None
        return {"pid": pid, "start_ticks": data[19], "pgid": os.getpgid(pid), "session_token": os.environ["GAZEBOTEACH_SESSION_TOKEN"]}
    except (OSError, KeyError, ValueError, IndexError):
        return None


def context(operation):
    config, record = prepared() if operation not in ("stop", "status") else (None, None)
    if not os.environ.get("GAZEBOTEACH_SESSION_TOKEN") or not os.environ.get("GAZEBOTEACH_SESSION_DIR"):
        raise RuntimeError("Use this manager inside the owned GazeboTeach session")
    session = Path(os.environ["GAZEBOTEACH_SESSION_DIR"]).resolve()
    session.mkdir(parents=True, exist_ok=True)
    return config, record, session


def managed(state):
    if not state.exists():
        return None
    value = json.loads(state.read_text(encoding="utf-8"))
    return value if identity(value["pid"]) == value else None


def stop(state):
    current = managed(state)
    if current:
        for sig, seconds in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            if identity(current["pid"]) != current:
                break
            os.kill(current["pid"], sig)
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline and identity(current["pid"]) == current:
                time.sleep(.03)
        if identity(current["pid"]) == current:
            raise RuntimeError("Owned RViz PID did not stop within its finite deadline")
    if state.exists():
        state.unlink()
    return {"stopped_pid": current["pid"] if current else None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=("start", "reopen", "stop", "status"), nargs="?", default="start")
    args = parser.parse_args()
    config, record, session = context(args.operation)
    state = session/"lesson24-rviz-pid.json"
    import fcntl
    with (session/"lesson24-rviz.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        if args.operation in ("stop", "reopen"):
            result = stop(state)
            if args.operation == "stop":
                print(json.dumps(result), flush=True)
                return
        current = managed(state)
        if args.operation == "status":
            print(json.dumps({"running": bool(current), "identity": current}), flush=True)
            return
        if current:
            print(json.dumps({"already_running": True, "identity": current}), flush=True)
            return
        if not os.environ.get("DISPLAY"):
            raise RuntimeError("Actual RViz requires the current graphical DISPLAY")
        template = (Path.cwd()/PACKAGE/"config/arm.rviz").read_text(encoding="utf-8")
        fixed = config["tf_prefix"]+"/world" if config["tf_prefix"] else "world"
        text = template.replace("Fixed Frame: world", "Fixed Frame: "+fixed).replace('TF Prefix: ""', 'TF Prefix: '+json.dumps(config["tf_prefix"]))
        target = session/"lesson24-rendered.rviz"
        target.write_bytes(text.encode("utf-8"))
        environment = os.environ.copy()
        # RViz 1.14.26 supports this child-only flag for its ROS 1 EOL modal.
        # It does not change the host's ROS configuration or renderer settings.
        environment["DISABLE_ROS1_EOL_WARNINGS"] = "1"
        with (session/"lesson24-rviz.log").open("ab", buffering=0) as log:
            child = subprocess.Popen(["rviz", "-d", str(target), "__name:=rviz", "__ns:=/lesson24"], stdout=log, stderr=subprocess.STDOUT, env=environment)
        deadline, own = time.monotonic()+4, None
        try:
            while time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError("Actual RViz exited early; see "+str(session/"lesson24-rviz.log"))
                own = identity(child.pid)
                if own:
                    break
                time.sleep(.03)
            if not own:
                raise RuntimeError("Actual RViz process identity was not observable")
            temporary = state.with_suffix(".tmp")
            temporary.write_text(json.dumps(own), encoding="utf-8")
            os.replace(str(temporary), str(state))
            time.sleep(.3)
            if child.poll() is not None:
                raise RuntimeError("Actual RViz failed during GUI startup; see its owned log")
            print(json.dumps({"started": True, "identity": own, "fixed_frame": fixed,
                "tf_prefix": config["tf_prefix"], "configuration": str(target),
                "note": "A running PID is not proof of RobotModel/TF rendering; inspect the actual window."}), flush=True)
        except BaseException:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=1)
            raise


if __name__ == "__main__":
    main()
