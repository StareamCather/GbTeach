#!/usr/bin/env python3
"""Generate only A43 private Action interfaces; preserve personal control code."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    workspace=Path.cwd().resolve();ros_ws=(workspace/'ros_ws').resolve()
    if workspace not in ros_ws.parents:raise RuntimeError('Private ROS workspace must remain inside this exercise')
    package=ros_ws/'src/gazeboteach_workshop43'
    for path in (workspace/'world.sdf',package/'package.xml',package/'launch/lab.launch'):ET.parse(str(path))
    for path in package.glob('scripts/*.py'):path.chmod(path.stat().st_mode|0o111)
    command=['catkin_make','-C',str(ros_ws),'-j2','-l2',
             '-DPYTHON_EXECUTABLE=/usr/bin/python3','-DCMAKE_BUILD_TYPE=Release',
             '-DCATKIN_ENABLE_TESTING=OFF','-DCATKIN_WHITELIST_PACKAGES=gazeboteach_workshop43']
    subprocess.run(command,check=True,timeout=115)
    generated=ros_ws/'devel/share/gazeboteach_workshop43/msg'
    # ROS_PACKAGE_PATH points to the private source package. Mirror only actual
    # catkin-generated schemas there so raw rosmsg introspection works without
    # activating an entire devel environment or changing global CMAKE_PREFIX_PATH.
    mirror=package/'msg';mirror.mkdir(exist_ok=True)
    names=['MoveSlideAction','MoveSlideActionGoal','MoveSlideActionResult','MoveSlideActionFeedback',
           'MoveSlideGoal','MoveSlideResult','MoveSlideFeedback']
    message_hashes={}
    for name in names:
        source=generated/(name+'.msg')
        if not source.is_file():raise RuntimeError('Catkin did not generate '+str(source))
        shutil.copyfile(str(source),str(mirror/source.name))
        message_hashes[source.name]=digest(source)
    environment=dict(os.environ)
    python_path=ros_ws/'devel/lib/python3/dist-packages'
    environment['PYTHONPATH']=str(python_path)+os.pathsep+environment.get('PYTHONPATH','')
    probe='from gazeboteach_workshop43.msg import MoveSlideAction,MoveSlideGoal; import json; print(json.dumps({"action_type":MoveSlideAction._type,"action_md5":MoveSlideAction._md5sum,"goal_type":MoveSlideGoal._type,"goal_md5":MoveSlideGoal._md5sum}))'
    interfaces=json.loads(subprocess.check_output(['/usr/bin/python3','-c',probe],text=True,env=environment,timeout=8))
    raw_action=subprocess.check_output(['rosmsg','show','-r','gazeboteach_workshop43/MoveSlideAction'],text=True,env=environment,timeout=8)
    raw_goal=subprocess.check_output(['rosmsg','show','-r','gazeboteach_workshop43/MoveSlideGoal'],text=True,env=environment,timeout=8)
    rosmsg_md5=subprocess.check_output(['rosmsg','md5','gazeboteach_workshop43/MoveSlideAction'],text=True,env=environment,timeout=8).strip()
    if 'target_m' not in raw_goal or 'timeout_wall_s' not in raw_goal or rosmsg_md5!=interfaces['action_md5']:
        raise RuntimeError('Raw rosmsg introspection differs from the actual generated Python Action interface')
    record={**interfaces,'action_definition_sha256':digest(package/'action/MoveSlide.action'),
        'generated_schema_sha256':message_hashes,'python_path':str(python_path),
        'rosmsg_raw_action':raw_action,'rosmsg_raw_goal':raw_goal,'rosmsg_action_md5':rosmsg_md5,
        'compiler_command':command,'scope':'Private catkin workspace and derived msg mirror only; student.py/instrument.py/reference.py preserved'}
    output=workspace/'build';output.mkdir(exist_ok=True)
    (output/'action-build.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    subprocess.run(['rosparam','set','/use_sim_time','true'],check=True,timeout=8)
    print(json.dumps({'prepared':True,'action_type':record['action_type'],'action_md5':record['action_md5'],
        'raw_rosmsg_verified':True,'personal_control_files_preserved':True}),flush=True)


if __name__=='__main__':main()
