#!/usr/bin/env python3
"""Actuate and observe a real model; no ROS or third-party Python dependency."""
import json
import re
import subprocess
import time


def command(*arguments):
    result = subprocess.run(list(arguments), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=7)
    if result.returncode:
        raise RuntimeError("{}: {}".format(" ".join(arguments), result.stderr.strip()))
    return result.stdout


def pose():
    result = command("gz", "model", "-m", "diagnostic_block", "-p")
    values = re.findall(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", result)
    if len(values) != 6:
        raise RuntimeError("Expected six pose coordinates, got: " + result)
    return [float(value) for value in values]


def main():
    paused = False
    try:
        command("gz", "world", "-w", "diagnostic", "-p", "1")
        paused = True
        command("gz", "model", "-m", "diagnostic_block", "-x", "0", "-y", "0", "-z", "2.4")
        raised = pose()
        if abs(raised[2] - 2.4) > 0.03:
            raise RuntimeError("Model did not accept raised pose: " + str(raised))
        command("gz", "world", "-w", "diagnostic", "-p", "0")
        paused = False
        deadline = time.monotonic() + 12
        settled = None
        while time.monotonic() < deadline:
            sample = pose()
            if abs(sample[2] - 0.2) < 0.035:
                settled = sample
                break
            time.sleep(0.15)
        if settled is None:
            raise RuntimeError("Block did not fall onto the floor within deadline")
        print(json.dumps({"passed": True, "raised_pose": raised, "settled_pose": settled,
                          "expected_center_z": 0.2}, ensure_ascii=False))
    finally:
        if paused:
            command("gz", "world", "-w", "diagnostic", "-p", "0")


if __name__ == "__main__":
    main()
