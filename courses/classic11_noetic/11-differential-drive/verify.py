#!/usr/bin/env python3
import json
import sys
from instrument import experiment

if __name__ == "__main__":
    try:
        print(json.dumps(experiment(), ensure_ascii=False), flush=True)
    except Exception as error:
        print("FAIL: "+type(error).__name__+": "+str(error), file=sys.stderr, flush=True)
        sys.exit(1)
