#!/usr/bin/env python3
"""Place the real body once, zero velocity; ODE calculates subsequent motion."""
import argparse
import json
import math
from config import prepared
from lab import Client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=float, default=1.4)
    parser.add_argument("--x", type=float, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.height) or not 0.6 <= args.height <= 3:
        parser.error("center height must be within [0.6,3] m")
    if not math.isfinite(args.x) or not -2 <= args.x <= 2:
        parser.error("x must be within [-2,2] m; x=1.5 misses the bumper")
    prepared()
    client, initial = Client(), None
    try:
        initial = client.physics().pause
        client.pause(True)
        client.release(args.height, args.x)
    finally:
        try:
            if initial is not None:
                client.pause(initial)
        finally:
            client.close()
    print(json.dumps({"released_center_m": [args.x, 0, args.height], "zero_initial_twist": True,
                      "paused": initial, "note": "ODE controls motion. Resume if originally paused."}, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
