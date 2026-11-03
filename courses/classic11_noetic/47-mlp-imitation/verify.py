import sys
import traceback
from lab import train_and_evaluate

if __name__=='__main__':
    try: train_and_evaluate(student=False)
    except Exception:
        traceback.print_exc(); sys.exit(1)
