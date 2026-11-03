#!/usr/bin/env python3
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
package=Path('ros_ws/src/gazeboteach_workshop42')
ET.parse(str(package/'package.xml'))
ET.parse(str(package/'launch/lab.launch'))
for path in Path('ros_ws/src').glob('*/scripts/*.py'):
    path.chmod(path.stat().st_mode | 0o111)
ET.parse('world.sdf')
subprocess.run(['rosparam','set','/use_sim_time','true'], check=True, timeout=8)
print('Private A42 XML/package scripts prepared; personal instrument.py and student.py preserved')
