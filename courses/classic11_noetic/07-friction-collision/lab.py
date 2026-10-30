#!/usr/bin/env python3
"""Read current contact predictions or deliberately reset and run this rig."""
import argparse
import json
import sys
from pathlib import Path

from contacts import configuration, predictions
from instrument import experiment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("predict", "experiment"))
    parser.add_argument("--report", help="把预测或真实测量存为JSON")
    args = parser.parse_args()
    result = predictions(configuration()) if args.action == "predict" else experiment()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.report:
        target = Path(args.report).expanduser().resolve()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("已保存：" + str(target))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("FAIL：" + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
