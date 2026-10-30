#!/usr/bin/env python3
import argparse
import json
import shlex
import sys
import uuid
from config import read_parameters, read_world
from instrument import OFFICIAL, WORKSPACE, entity, experiment, inspect, rpc, send, topic_info, wait_ticket, watch


def main():
    parser = argparse.ArgumentParser(description="真实Classic Transport消息滑门实验")
    parser.add_argument("action", choices=("check-config", "status", "entity", "send", "wait", "watch", "open", "close", "demo", "cli-example", "query", "info"))
    parser.add_argument("--target")
    parser.add_argument("--client")
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--nonce")
    parser.add_argument("--run")
    parser.add_argument("--epoch", type=int)
    parser.add_argument("--verb", default="set_target")
    parser.add_argument("--omit-target", action="store_true")
    parser.add_argument("--topic", default="~/transport_lab/command")
    parser.add_argument("--expect-type", default="gazebo.msgs.Request")
    parser.add_argument("--wait", choices=("received", "completed"), default="completed")
    parser.add_argument("--ticket")
    parser.add_argument("--timeout", type=float, default=6)
    parser.add_argument("--seconds", type=float, default=2)
    parser.add_argument("--request", default="gazeboteach_transport_snapshot")
    args = parser.parse_args()
    if args.action == "check-config":
        read_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf")
        result = {"passed": True, "parameters": read_parameters(WORKSPACE/"parameters.json"), "simulator_started": False}
    elif args.action == "status":
        result = inspect()
    elif args.action == "entity":
        result = entity()
    elif args.action in ("send", "open", "close"):
        target = "0" if args.action == "close" else args.target
        result = send(target=target, client=args.client, seq=args.seq, nonce=args.nonce, run=args.run, epoch=args.epoch,
                      verb=args.verb, omit_target=args.omit_target, topic=args.topic, expect_type=args.expect_type,
                      wait=args.wait, ticket_path=args.ticket, timeout=args.timeout)
    elif args.action == "wait":
        result = wait_ticket(args.ticket, args.wait, args.timeout)
    elif args.action == "watch":
        result = watch(args.seconds)
    elif args.action == "query":
        result = rpc(args.request, args.nonce, min(args.timeout, 3))
    elif args.action == "info":
        info = topic_info(args.topic)
        result = {"passed": bool(info["actual_type"]), "topic_info": info}
    elif args.action == "cli-example":
        current = inspect()["state"]
        client, nonce, attempt = "terminal-"+uuid.uuid4().hex[:10], uuid.uuid4().hex, uuid.uuid4().hex
        text = 'id: 1 request: "set_target" data: "%s %s %s %s %s" dbl_data: %.17g' % (current["run_id"], client, nonce, current["epoch"], attempt, current["parameters"]["open_target_m"])
        command = "gz topic -w default -p '~/transport_lab/command' -m "+shlex.quote(text)
        result = {"passed": True, "command_executed": False, "command": command,
                  "note": "在本课终端执行；先用echo观察ACK，新示例有新nonce，CLI退出0不代表真实完成"}
    else:
        result = experiment()
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(",", ":")), flush=True)
    return 0 if result.get("passed", True) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({"passed": False, "error": type(error).__name__+": "+str(error)}, ensure_ascii=False, allow_nan=False), flush=True)
        sys.exit(1)
