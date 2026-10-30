#!/usr/bin/env python3
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from config import PHASES, read_parameters, read_world
from instrument import OFFICIAL, WORKSPACE, experiment, inspect, replay_calibration, run_phase


def main():
    parser = argparse.ArgumentParser(description="真实Classic差速轮式小车实验")
    parser.add_argument("action", choices=("check-config", "inspect", "run", "demo", "set-surface", "replay-calibration"))
    parser.add_argument("value", nargs="?")
    args = parser.parse_args()
    if args.action == "check-config":
        result = {"passed": True, "parameters": read_parameters(WORKSPACE/"parameters.json"),
                  "world": read_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf"), "simulator_started": False}
    elif args.action == "set-surface":
        if args.value not in ("normal", "slip"):
            parser.error("set-surface requires normal or slip")
        path = WORKSPACE/"world.sdf"
        read_world(path, OFFICIAL/"world.sdf")
        tree = ET.parse(str(path))
        for name in ("left_wheel", "right_wheel"):
            node = tree.getroot().find("world/model[@name='differential_car']/link[@name='%s']/collision/surface/friction/ode" % name)
            for key in ("mu", "mu2"):
                node.find(key).text = ".8" if args.value == "normal" else ".001"
        temporary = path.with_suffix(".tmp")
        tree.write(str(temporary), encoding="utf-8", xml_declaration=True)
        read_world(temporary, OFFICIAL/"world.sdf")
        os.replace(str(temporary), str(path))
        result = {"surface_mode": args.value, "reload_required": True, "simulator_changed": False,
                  "message": "已修改个人world.sdf，须点击重新加载项目；当前实例仍保持原表面"}
    elif args.action == "inspect":
        result = inspect()
    elif args.action == "run":
        if args.value not in PHASES:
            parser.error("run requires straight, spin or arc")
        result = run_phase(args.value)
    elif args.action == "replay-calibration":
        result = replay_calibration(.48 if args.value is None else float(args.value))
    else:
        result = experiment()
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("FAIL: "+type(error).__name__+": "+str(error), file=sys.stderr, flush=True)
        sys.exit(1)
