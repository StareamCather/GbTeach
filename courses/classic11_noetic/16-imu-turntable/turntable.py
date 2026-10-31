#!/usr/bin/env python3
"""Set a true initial pose/twist once; no continuous trajectory enforcement."""
import argparse
import json
import math
from config import prepared
from lab import Client, state_values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=.6)
    parser.add_argument("--tilt", type=float, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not -1.5 <= args.speed <= 1.5:
        parser.error("speed must be within[-1.5,1.5]rad/s")
    if not math.isfinite(args.tilt) or not -60 <= args.tilt <= 60:
        parser.error("tilt must be within[-60,60]degrees")
    if args.speed != 0 and args.tilt != 0:
        parser.error("This principal-axis fixture supports tilted rest or upright spin; set speed0 before tilting")
    prepared()
    client, paused = Client(), None
    try:
        paused = client.physics().pause
        client.pause(True)
        client.setup(roll=math.radians(args.tilt), speed=args.speed)
        result = {"initial_state": state_values(client.state()), "paused": paused,
                  "note": "One actual initialization at (0,0,1), yaw0; ODE integrates later motion. Resume if originally paused."}
    finally:
        try:
            if paused is not None:
                client.pause(paused)
        finally:
            client.close()
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
