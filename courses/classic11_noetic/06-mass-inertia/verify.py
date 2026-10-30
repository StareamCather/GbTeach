#!/usr/bin/env python3
"""Trusted source verifier; current cwd supplies all learner parameters/assets."""
import sys

from instrument import experiment


if __name__ == "__main__":
    try:
        experiment()
    except Exception as error:
        print("FAIL：" + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
