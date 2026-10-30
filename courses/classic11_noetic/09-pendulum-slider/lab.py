#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import sys
import time
from kinematics import configuration,predictions
from instrument import binary_path,experiment,invoke
import transport as gz

def main():
    p=argparse.ArgumentParser(description="预测、读取原生关节，或重置后测被动机构")
    p.add_argument("action",choices=("predict","inspect","experiment"));p.add_argument("--report");args=p.parse_args()
    if args.action=="experiment":result=experiment()
    else:
        config=configuration()
        if args.action=="predict":result=predictions(config)
        else:
            gz.DEADLINE=time.monotonic()+10;result=invoke(binary_path(),config,"inspect")
            if result["returncode"]:raise RuntimeError(result["stderr"])
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if args.report:
        path=Path(args.report).expanduser().resolve();path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8");print("已保存："+str(path))

if __name__=="__main__":
    try:main()
    except Exception as error:
        print("FAIL："+str(error),file=sys.stderr,flush=True);sys.exit(1)
