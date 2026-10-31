#!/usr/bin/env python3
"""Real WorldPlugin factory, native falling models and scoped reclamation."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    started=time.monotonic()
    try:
        from lab import Lab
        result=Lab().verify()
    except Exception as error:
        result={'passed':False,'error':type(error).__name__+': '+str(error)}
    result['wall_seconds']=time.monotonic()-started
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':sys.exit(main())
