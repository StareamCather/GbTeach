#!/usr/bin/env python3
"""Inspect the personal exercise and read only this owned session's evidence."""
import argparse,json,os
from pathlib import Path
from scene import validate

def main():
 p=argparse.ArgumentParser();p.add_argument('action',choices=['inspect','evidence','image']);a=p.parse_args()
 if a.action=='inspect': print(json.dumps(validate(),ensure_ascii=False,indent=2))
 else:
  base=Path(os.environ['GAZEBOTEACH_SESSION_DIR'])
  for name in ['physics.json','camera.json','wind.json']:
   path=base/name
   if path.exists():print(name,path.read_text(encoding='utf-8'))
  if (base/'camera.ppm').exists():print('ACTUAL_CAMERA_FRAME:',base/'camera.ppm')
if __name__=='__main__':main()
