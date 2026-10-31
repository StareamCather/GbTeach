#!/usr/bin/env python3
"""Compile the offline SDK examples with a lock per immutable source cache key."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
import xml.etree.ElementTree as ET
from config import SOURCES, parameters, sha


def main():
    workspace=Path.cwd().resolve();value=parameters();build=workspace/'build'
    if workspace not in build.resolve().parents:raise ValueError('Build path leaves the personal workspace')
    (build/'lib').mkdir(parents=True,exist_ok=True)
    world=workspace/'world.sdf';xml=ET.parse(str(world));root=xml.find('world')
    if root is None or root.get('name')!='lesson29':raise ValueError('Keep the lesson29 world identity')
    plugin=root.find("model[@name='beacon']/link/visual[@name='indicator']/plugin[@name='appearance']")
    if plugin is None or plugin.get('filename')!='libgazeboteach_l29_visual.so':raise ValueError('Keep the VisualPlugin identity')
    for key,text in [('color',value['color']),('visual_scale',format(value['visual_scale'],'.17g')),
                     ('config_sha256',sha(workspace/'parameters.json'))]:
        plugin.find(key).text=text
    xml.write(str(world),encoding='utf-8',xml_declaration=True)
    version=subprocess.check_output(['pkg-config','--modversion','gazebo'],text=True,timeout=5).strip()
    if not version.startswith('11.'):raise RuntimeError('Use the actual Gazebo Classic 11 SDK')
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','gazebo','OGRE','OGRE-Paging','OGRE-Terrain','OGRE-RTShaderSystem','Qt5Widgets'],text=True,timeout=5).strip()
    compiler=subprocess.check_output(['g++','--version'],text=True,timeout=5).splitlines()[0]
    sources={name:sha(workspace/name) for name in SOURCES}
    key=hashlib.sha256((sources['visual_plugin.cc']+sources['observer.cc']+version+flags+compiler).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/visual-plugin'/key[:24]
    cache.mkdir(parents=True,exist_ok=True);commands=[]
    # An atomic rename alone is insufficient: a second compiler must not replace
    # the same ELF after the first learner already mapped its inode into gzserver.
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        for source,name,extra,target in [
            ('visual_plugin.cc','libgazeboteach_l29_visual.so',['-shared','-fPIC','-DGZTEACH_SOURCE_SHA="'+sources['visual_plugin.cc']+'"'],build/'lib/libgazeboteach_l29_visual.so'),
            ('observer.cc','visual_observer',[],build/'visual_observer')]:
            output=cache/name
            if not output.is_file():
                temporary=cache/('compile-'+uuid.uuid4().hex)
                command=['g++','-std=c++17','-O2','-Wno-deprecated-declarations']+extra+[str(workspace/source),'-o',str(temporary)]+shlex.split(flags)
                if source=='observer.cc':command.append('-ldl')
                else:command.append('-lgazebo_gui')
                commands.append(command)
                try:
                    result=subprocess.run(command,text=True,capture_output=True,timeout=65)
                    if result.returncode:raise RuntimeError('Actual compiler failed:\n'+result.stdout+'\n'+result.stderr)
                    os.replace(str(temporary),str(output))
                finally:
                    if temporary.exists():temporary.unlink()
            if target.exists() or target.is_symlink():target.unlink()
            target.symlink_to(output)
    record={'parameters':value,
            'config_sha256':sha(workspace/'parameters.json'),'source_sha256':sources,
            'world_sha256':sha(world),'gazebo_version':version,'compiler':compiler,'compiler_commands':commands,
            'cache_key':key,'library_sha256':sha(build/'lib/libgazeboteach_l29_visual.so'),
            'observer_sha256':sha(build/'visual_observer')}
    (build/'visual_lab.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'prepared':True,'parameters':value,'cache':str(cache)}))


if __name__=='__main__':main()
