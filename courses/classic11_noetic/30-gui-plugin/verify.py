#!/usr/bin/env python3
"""Verify actual Qt controls against independent native world observations."""
import argparse
import json
from pathlib import Path
import time
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--report',type=Path);args=p.parse_args();start=time.monotonic()
    try:
        from lab import Lab
        result=Lab().verify()
    except Exception as error:result={'passed':False,'error':type(error).__name__+': '+str(error)}
    result['wall_seconds']=time.monotonic()-start
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,allow_nan=False));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
