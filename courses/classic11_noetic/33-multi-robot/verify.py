#!/usr/bin/env python3
"""Finite physical isolation verification; preserve full observations in build/."""
import fcntl
import json
from pathlib import Path
import signal
from lab import Client, handle_signal


def main():
    signal.signal(signal.SIGTERM, handle_signal)
    client = Client()
    with (Path("build")/"lab.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = client.exercise()
        output = Path("build/verify_report.json")
        output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "parameters": result["parameters"], "report": str(output),
        "phases": [{"phase": item["phase"], "metrics": item["metrics"]} for item in result["phases"]]}, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
