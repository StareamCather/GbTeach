#!/usr/bin/env python3
import argparse
import json
import sys
from config import edit_world, read_world, resources
from instrument import OFFICIAL, WORKSPACE, experiment, inspect, view


def main():
    parser = argparse.ArgumentParser(description="真实Classic场景展馆")
    parser.add_argument("action", choices=("check-config", "diagnose", "inspect", "observe", "demo", "set-height", "set-cycle", "set-daylight", "view"))
    parser.add_argument("--value", type=float)
    parser.add_argument("--station", choices=("overview", "materials", "specular", "lights", "terrain", "actors"), default="overview")
    parser.add_argument("--timeout", type=float, default=4)
    parser.add_argument("--activate", action="store_true", help="仅新GUI初始化时激活本会话窗口并刷新一次尺寸")
    args = parser.parse_args()
    if args.action in ("check-config", "diagnose"):
        result = {"passed": True, "parameters": read_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf"),
                  "assets": resources(WORKSPACE), "simulator_started": False}
    elif args.action.startswith("set-"):
        result = edit_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf", args.action[4:], args.value)
    elif args.action == "inspect":
        result = inspect()
        result["passed"] = True
    elif args.action == "view":
        result = view(args.station, args.timeout, activate=args.activate)
    else:
        result = experiment()
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result.get("passed", True) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({"passed": False, "error": type(error).__name__+": "+str(error)}, ensure_ascii=False, allow_nan=False), flush=True)
        sys.exit(1)
