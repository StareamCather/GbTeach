#!/usr/bin/env python3
"""Run independently owned, real headless Gazebo trials; save JSON and CSV."""
import argparse
import json
from pathlib import Path
from lab import Lab, write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('action',choices=('run',))
    parser.add_argument('--report',type=Path); args=parser.parse_args()
    result=Lab().batch()
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True); write_json(args.report,result)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try: main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False)); raise SystemExit(1)
