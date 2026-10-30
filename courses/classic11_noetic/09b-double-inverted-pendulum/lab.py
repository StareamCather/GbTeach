#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import sys
from instrument import WORKSPACE, experiment, inspect, observe, unit_check


def main():
    parser=argparse.ArgumentParser(description="真实倒立摆实验；参考演示不能标记作业通过")
    parser.add_argument("action",choices=("inspect","unit-check","mode","demo-check","observe"))
    parser.add_argument("mode",nargs="?",choices=("student","reference_demo"))
    parser.add_argument("--seconds",type=float,default=3)
    args=parser.parse_args()
    if args.action=="mode":
        if not args.mode:
            parser.error("mode requires student or reference_demo")
        (WORKSPACE/"runtime.json").write_text(json.dumps({"mode":args.mode},indent=2)+"\n",encoding="utf-8")
        print("模式已设为%s。learner.py已保留；必须点击重载项目才会执行新模式。"%args.mode)
    elif args.action=="demo-check":
        experiment("reference_demo")
        print("参考演示验证完成；这不是学生作业通过。")
    elif args.action=="observe":
        print(json.dumps(observe(args.seconds),ensure_ascii=False,indent=2))
    else:
        print(json.dumps(inspect() if args.action=="inspect" else unit_check(),ensure_ascii=False,indent=2))


if __name__=="__main__":
    try:
        main()
    except Exception as error:
        print("FAIL："+str(error),file=sys.stderr,flush=True)
        sys.exit(1)
