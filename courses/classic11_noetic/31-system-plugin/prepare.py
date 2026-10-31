#!/usr/bin/env python3
"""Offline SDK build: immutable source cache, compile lock, personal config."""
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
    root=Path.cwd().resolve();value=parameters();build=root/'build'
    if root not in build.resolve().parents:raise ValueError('Build path leaves personal workspace')
    (build/'lib').mkdir(parents=True,exist_ok=True)
    world=root/'world.sdf';xml=ET.parse(str(world));element=xml.find('world')
    if element is None or element.get('name')!=WORLD:raise ValueError('Keep the lesson31 world identity')
    if list(xml.iter('plugin')):raise ValueError('SystemPlugin must be loaded by gzserver -s, never SDF plugin')
    if element.find("model[@name='falling_cube']") is None:raise ValueError('Keep the real falling_cube model')
    config=build/'system_plugin.cfg'
    config.write_text('world '+WORLD+'\ngravity_m_s2 '+format(value['gravity_m_s2'],'.17g')+
                      '\nrecord_every_steps '+str(value['record_every_steps'])+
                      '\nconfig_sha256 '+sha(root/'parameters.json')+'\n',encoding='utf-8')
    version=subprocess.check_output(['pkg-config','--modversion','gazebo'],text=True,timeout=5).strip()
    if not version.startswith('11.'):raise RuntimeError('Use the actual Gazebo Classic 11 SDK')
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True,timeout=5).strip()
    compiler=subprocess.check_output(['g++','--version'],text=True,timeout=5).splitlines()[0]
    sources={name:sha(root/name) for name in SOURCES}
    key=hashlib.sha256((sources['system_plugin.cc']+sources['observer.cc']+version+flags+compiler).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/system-plugin'/key[:24]
    cache.mkdir(parents=True,exist_ok=True);commands=[]
    # Never replace an ELF inode already mapped by a running personal server.
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        for source,name,extra,target in [
            ('system_plugin.cc','libgazeboteach_l31_system.so',['-shared','-fPIC','-DGZTEACH_SOURCE_SHA="'+sources['system_plugin.cc']+'"'],build/'lib/libgazeboteach_l31_system.so'),
            ('observer.cc','system_observer',[],build/'system_observer')]:
            output=cache/name
            if not output.is_file():
                temporary=cache/('compile-'+uuid.uuid4().hex)
                command=['g++','-std=c++17','-O2','-Wno-deprecated-declarations']+extra+[str(root/source),'-o',str(temporary)]+shlex.split(flags)
                if source=='observer.cc':command.append('-ldl')
                commands.append(command)
                try:
                    result=subprocess.run(command,text=True,capture_output=True,timeout=65)
                    if result.returncode:raise RuntimeError('Actual compiler failed:\n'+result.stdout+'\n'+result.stderr)
                    os.replace(str(temporary),str(output))
                finally:
                    if temporary.exists():temporary.unlink()
            if target.exists() or target.is_symlink():target.unlink()
            target.symlink_to(output)
    record={'parameters':value,'config_sha256':sha(root/'parameters.json'),'source_sha256':sources,
            'world_sha256':sha(world),'plugin_config_sha256':sha(config),'gazebo_version':version,
            'compiler':compiler,'compiler_commands':commands,'cache_key':key,
            'library_sha256':sha(build/'lib/libgazeboteach_l31_system.so'),'observer_sha256':sha(build/'system_observer')}
    (build/'system_lab.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'prepared':True,'parameters':value,'cache':str(cache),'load_method':'gzserver -u -s libgazeboteach_l31_system.so world.sdf'}))


if __name__=='__main__':main()
