#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys
source=Path(os.environ.get('GAZEBOTEACH_LESSON_SOURCE',Path(__file__).resolve().parent)).resolve()
sys.path.insert(0,str(source))
from instrument import verify
if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False,indent=2,allow_nan=False))
