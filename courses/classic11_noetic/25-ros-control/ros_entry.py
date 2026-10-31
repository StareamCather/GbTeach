#!/usr/bin/env python3
import os,sys
from pathlib import Path
from config import overlay,prepared
w=Path(__file__).resolve().parent
_,record=prepared(w)
e=overlay(w)
operation=sys.argv[1] if len(sys.argv)>1 else 'start'
if operation not in ('start','nodes','params','find'):raise ValueError('Use start/nodes/params/find')
args=['rospack','find','gazeboteach_l25'] if operation=='find' else ['roslaunch']+{'start':['--wait'],'nodes':['--nodes'],'params':['--dump-params']}[operation]+['gazeboteach_l25','servo.launch','description_file:='+record['generated_urdf_path'],'controllers_file:='+record['controllers_path']]
os.chdir(str(w));os.execvpe(args[0],args,e)
