#!/usr/bin/env python3
"""Bounded, physical ros_control teaching checks; supports an operator report."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report",type=Path)
    args=parser.parse_args()
    result={"passed":False}
    start=time.monotonic()
    lab=None
    try:
        from lab import Lab
        lab=Lab()
        result=lab.verify()
    except Exception as error:
        result.update(passed=False,error=type(error).__name__+": "+str(error))
    finally:
        if lab:lab.close()
    result["wall_seconds"]=time.monotonic()-start
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    return 0 if result["passed"] else 1


if __name__=="__main__":sys.exit(main())
