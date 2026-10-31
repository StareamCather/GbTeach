#!/usr/bin/env python3
"""Compile the standalone observer from the installed Classic11 SDK offline."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
import xml.etree.ElementTree as ET
from config import SOURCES, WORLD, parameters, sha


def main():
    root=Path.cwd().resolve();value=parameters();build=root/'build';build.mkdir(exist_ok=True)
    xml=ET.parse(str(root/'world.sdf'));world=xml.find('world')
    if world is None or world.get('name')!=WORLD or list(xml.iter('plugin')):
        raise ValueError('Use the standalone lesson32 world without plugins')
    cube=world.find("model[@name='falling_cube']")
    if cube is None:raise ValueError('Keep falling_cube')
    links=cube.findall('link')
    if len(links)!=1 or links[0].get('name')!='body':raise ValueError('This lesson requires the single canonical body link')
    if any(float(v)!=0 for v in links[0].findtext('pose','0 0 0 0 0 0').split()):
        raise ValueError('Keep the body link initial relative pose at identity; effective native playback follows this canonical link')
    version=subprocess.check_output(['pkg-config','--modversion','gazebo'],text=True,timeout=5).strip()
    if not version.startswith('11.'):raise RuntimeError('Actual Gazebo Classic11 development SDK required')
    # gazebo.pc omits util even though LogPlay lives in libgazebo_util.
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True,timeout=5).strip()+' -lgazebo_util -ldl'
    compiler=subprocess.check_output(['g++','--version'],text=True,timeout=5).splitlines()[0]
    sources={name:sha(root/name) for name in SOURCES}
    key=hashlib.sha256((sources['observer.cc']+version+flags+compiler).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/native-log'/key[:24]
    cache.mkdir(parents=True,exist_ok=True);commands=[]
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX);output=cache/'log_observer'
        if not output.is_file():
            temporary=cache/('compile-'+uuid.uuid4().hex)
            command=['g++','-std=c++17','-O2','-Wno-deprecated-declarations',str(root/'observer.cc'),'-o',str(temporary)]+shlex.split(flags)
            commands.append(command)
            try:
                result=subprocess.run(command,text=True,capture_output=True,timeout=75)
                if result.returncode:raise RuntimeError('Actual compiler failed:\n'+result.stdout+'\n'+result.stderr)
                os.replace(str(temporary),str(output))
            finally:
                if temporary.exists():temporary.unlink()
        target=build/'log_observer'
        if target.exists() or target.is_symlink():target.unlink()
        target.symlink_to(output)
    record={'parameters':value,'config_sha256':sha(root/'parameters.json'),'source_sha256':sources,
            'world_sha256':sha(root/'world.sdf'),'gazebo_version':version,'compiler':compiler,
            'compiler_commands':commands,'cache_key':key,'observer_sha256':sha(build/'log_observer')}
    (build/'log_lab.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'prepared':True,'parameters':value,'cache':str(cache),'load_method':'gzserver -u world.sdf'}))


if __name__=='__main__':main()
