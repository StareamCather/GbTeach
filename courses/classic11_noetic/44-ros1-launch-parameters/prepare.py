#!/usr/bin/env python3
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
package=Path('ros_ws/src/gazeboteach_workshop44')
for path in (Path('world.sdf'),package/'package.xml',package/'launch/lab.launch'):ET.parse(str(path))
for path in package.glob('scripts/*.py'):path.chmod(path.stat().st_mode|0o111)
subprocess.run(['rosparam','set','/use_sim_time','true'],check=True,timeout=8)
print('A44 source-only private package prepared; personal instrument/student/reference/config preserved')
