#!/usr/bin/env python3
import argparse
import json
import sys
from config import PHASES, read_parameters
from instrument import WORKSPACE, experiment, inspect, run_phase

def main():
    parser = argparse.ArgumentParser(description="真实Classic原生PID电机实验")
    parser.add_argument("action", choices=("check-config", "inspect", "run", "demo"))
    parser.add_argument("phase", nargs="?", choices=[name for name, _ in PHASES])
    args = parser.parse_args()
    if args.action == "check-config":
        result = {"passed": True, "parameters": read_parameters(WORKSPACE/"parameters.json"), "simulator_started": False}
    elif args.action == "inspect":
        result = inspect()
    elif args.action == "run":
        if not args.phase:
            parser.error("run requires a bounded phase name")
        result = run_phase(args.phase)
    else:
        result = experiment()
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("FAIL: "+type(error).__name__+": "+str(error), file=sys.stderr, flush=True)
        sys.exit(1)
