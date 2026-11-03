import sys
import traceback
from lab import experiment

if __name__=='__main__':
    try:experiment(False)
    except Exception:
        traceback.print_exc();sys.exit(1)
