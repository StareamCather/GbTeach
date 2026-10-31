#!/usr/bin/env python3
"""MOC and compile an actual Qt GUIPlugin in a private immutable SDK cache."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
import xml.etree.ElementTree as ET
from config import SOURCES,parameters,sha

def execute(command,commands):
    commands.append(command);result=subprocess.run(command,text=True,capture_output=True,timeout=90)
    if result.returncode:raise RuntimeError('Actual MOC/compiler failed:\n'+result.stdout+'\n'+result.stderr)
    return result
def main():
    root=Path.cwd().resolve();value=parameters();build=root/'build';(build/'lib').mkdir(parents=True,exist_ok=True)
    xml=ET.parse(str(root/'world.sdf'));world=xml.find('world')
    if world is None or world.get('name')!='lesson30':raise ValueError('Keep the lesson30 world identity')
    plugin=world.find("gui/plugin[@name='teaching_panel']")
    if plugin is None or plugin.get('filename')!='libgazeboteach_l30_gui.so':raise ValueError('Keep the GUIPlugin identity')
    for name,text in [('step_count',str(value['step_count'])),('display_decimals',str(value['display_decimals'])),('config_sha256',sha(root/'parameters.json'))]:plugin.find(name).text=text
    xml.write(str(root/'world.sdf'),encoding='utf-8',xml_declaration=True)
    version=subprocess.check_output(['pkg-config','--modversion','gazebo'],text=True,timeout=5).strip()
    if not version.startswith('11.'):raise RuntimeError('Use the actual Classic11 SDK')
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','gazebo','OGRE','OGRE-Paging','OGRE-Terrain','OGRE-RTShaderSystem','Qt5Widgets'],text=True,timeout=5).strip()
    compiler=subprocess.check_output(['g++','--version'],text=True,timeout=5).splitlines()[0]
    host_bins=subprocess.check_output(['pkg-config','--variable=host_bins','Qt5Core'],text=True,timeout=5).strip()
    moc_binary=str(Path(host_bins)/'moc')
    moc=subprocess.run([moc_binary,'-v'],text=True,capture_output=True,timeout=5);moc_version=(moc.stdout+moc.stderr).strip()
    if moc.returncode:raise RuntimeError('Qt5 moc is required: '+moc_version)
    sources={name:sha(root/name) for name in SOURCES}
    key=hashlib.sha256((''.join(sources[name] for name in ('gui_plugin.hh','gui_plugin.cc','observer.cc'))+version+flags+compiler+moc_version).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/gui-plugin'/key[:24];cache.mkdir(parents=True,exist_ok=True);commands=[]
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        library=cache/'libgazeboteach_l30_gui.so'
        if not library.is_file():
            moc_source=cache/('moc-'+uuid.uuid4().hex+'.cc');temporary=cache/('compile-'+uuid.uuid4().hex)
            try:
                execute([moc_binary,str(root/'gui_plugin.hh'),'-o',str(moc_source)],commands)
                execute(['g++','-std=c++17','-O2','-Wno-deprecated-declarations','-shared','-fPIC','-DGZTEACH_SOURCE_SHA="'+sources['gui_plugin.cc']+'"',str(root/'gui_plugin.cc'),str(moc_source),'-o',str(temporary)]+shlex.split(flags)+['-lgazebo_gui'],commands)
                os.replace(str(temporary),str(library))
            finally:
                for p in (moc_source,temporary):
                    if p.exists():p.unlink()
        observer=cache/'panel_observer'
        if not observer.is_file():
            temporary=cache/('compile-'+uuid.uuid4().hex)
            try:
                execute(['g++','-std=c++17','-O2',str(root/'observer.cc'),'-o',str(temporary)]+shlex.split(flags)+['-ldl'],commands)
                os.replace(str(temporary),str(observer))
            finally:
                if temporary.exists():temporary.unlink()
        for target,source in [(build/'lib/libgazeboteach_l30_gui.so',library),(build/'panel_observer',observer)]:
            if target.exists() or target.is_symlink():target.unlink()
            target.symlink_to(source)
    record={'parameters':value,'config_sha256':sha(root/'parameters.json'),'source_sha256':sources,'world_sha256':sha(root/'world.sdf'),
            'gazebo_version':version,'compiler':compiler,'moc_version':moc_version,'compiler_commands':commands,'cache_key':key,
            'library_sha256':sha(build/'lib/libgazeboteach_l30_gui.so'),'observer_sha256':sha(build/'panel_observer')}
    (build/'gui_lab.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'prepared':True,'parameters':value,'cache':str(cache),'moc':moc_version}))
if __name__=='__main__':main()
