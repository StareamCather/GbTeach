#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import sys
import time

from instrument import experiment, physics_info
from stability import configuration, predictions
import transport as gz


def main():
    parser = argparse.ArgumentParser(description="预测、读取实际physics，或重置后测接触几何与稀疏尾部波动")
    parser.add_argument("action", choices=("predict", "physics", "experiment"))
    parser.add_argument("--report")
    args = parser.parse_args()
    if args.action == "experiment":
        result = experiment()
    else:
        config = configuration()
        if args.action == "predict":
            result = predictions(config)
        else:
            gz.DEADLINE = time.monotonic()+8
            result = physics_info(config["world"])
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.report:
        path = Path(args.report).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("已保存："+str(path))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("FAIL："+str(error), file=sys.stderr, flush=True)
        sys.exit(1)
