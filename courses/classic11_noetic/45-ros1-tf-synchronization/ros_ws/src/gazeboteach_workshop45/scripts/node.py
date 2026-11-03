#!/usr/bin/env python3
import os
from pathlib import Path
import sys
workspace=Path(__file__).resolve().parents[4]
os.chdir(str(workspace));sys.path.insert(0,str(workspace))
from instrument import main
if __name__=='__main__':main()
