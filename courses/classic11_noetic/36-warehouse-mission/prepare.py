#!/usr/bin/env python3
"""Offline atomic native build keyed by code, actual headers and runtime DSOs."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import uuid
from config import SOURCES, sha, parameters


def output(command, timeout=20):
    return subprocess.check_output(command, text=True, timeout=timeout).strip()


def sdk_inputs(root, cflags, libs):
    # Ask the actual compiler for its transitive header set, including Boost,
    # protobuf, ignition and C++ headers. A version string alone is insufficient.
    inputs = {}
    for source in ('door_client.cc', 'door_plugin.cc'):
        dependency_text = output(['g++', '-std=c++17', '-M', str(root/source)] + cflags)
        dependencies = shlex.split(dependency_text.replace('\\\n', ' ').split(':', 1)[1])
        for filename in dependencies:
            path = Path(filename).resolve()
            if path.parent != root: inputs[str(path)] = sha(path)
    directories = [Path(flag[2:]) for flag in libs if flag.startswith('-L')]
    directories += [Path('/usr/lib')/output(['g++', '-print-multiarch']), Path('/usr/lib'), Path('/lib')/output(['g++', '-print-multiarch'])]
    libraries = []
    for flag in libs:
        if not flag.startswith('-l'): continue
        candidates = [directory/('lib'+flag[2:]+'.so') for directory in directories]
        found = next((p.resolve() for p in candidates if p.is_file()), None)
        if found is None:
            p = Path(output(['g++', '-print-file-name=lib'+flag[2:]+'.so']))
            if p.is_file(): found = p.resolve()
        if found is None: raise RuntimeError('Cannot fingerprint linked library: '+flag)
        libraries.append(found)
    for library in libraries:
        inputs[str(library)] = sha(library)
        result = subprocess.run(['ldd', str(library)], text=True, capture_output=True, timeout=10)
        if result.returncode: raise RuntimeError('Cannot inspect runtime dependencies: '+str(library))
        for line in result.stdout.splitlines():
            words = line.replace('=>', ' ').split()
            for word in words:
                if word.startswith('/') and Path(word).is_file():
                    path = Path(word).resolve(); inputs[str(path)] = sha(path)
    compiler = Path(shutil.which('g++')).resolve(); inputs[str(compiler)] = sha(compiler)
    return inputs


def main():
    import xml.etree.ElementTree as ET
    root=Path.cwd().resolve(); build=root/'build'; build.mkdir(exist_ok=True)
    value=parameters()
    if not os.environ.get('GAZEBOTEACH_SESSION_TOKEN') or not os.environ.get('ROS_MASTER_URI'):
        raise RuntimeError('Use the existing GazeboTeach private ROS session')
    version=output(['pkg-config','--modversion','gazebo'])
    exact=Path('/usr/bin/gzserver-11.15.1'); server=Path(shutil.which('gzserver') or '/missing')
    if version!='11.15.1' or not exact.is_file() or not server.is_file() or not os.path.samefile(str(exact),str(server)):
        raise RuntimeError('Exact Classic11.15.1 gzserver/SDK required')
    package=Path(output(['rospack','find','gazebo_plugins']))
    if ET.parse(str(package/'package.xml')).findtext('version')!='2.9.3':
        raise RuntimeError('Verified Noetic gazebo_plugins2.9.3 required')
    ros_plugins={}
    for name in ('libgazebo_ros_diff_drive.so','libgazebo_ros_laser.so','libgazebo_ros_bumper.so'):
        path=package.parent.parent/'lib'/name
        if not path.is_file(): raise RuntimeError('Missing actual ROS DSO: '+str(path))
        ros_plugins[name]={'path':str(path.resolve()),'sha256':sha(path)}
    probe=subprocess.run(['gazebo','--version'],text=True,capture_output=True,timeout=5)
    if probe.returncode not in (0,255) or '11.15.1' not in probe.stdout+probe.stderr:
        raise RuntimeError('Gazebo executable version differs')
    cflags=shlex.split(output(['pkg-config','--cflags','gazebo']))
    libs=shlex.split(output(['pkg-config','--libs','gazebo']))+['-ldl']
    sources={name:sha(root/name) for name in SOURCES}
    build_identity={'sources':sources,'sdk_inputs':sdk_inputs(root,cflags,libs),'cflags':cflags,'libs':libs,
                    'gazebo_version':version,'compiler':output(['g++','--version']).splitlines()[0]}
    key=hashlib.sha256(json.dumps(build_identity,sort_keys=True).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/warehouse'/key
    cache.mkdir(parents=True,exist_ok=True); products={}; commands=[]
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        previous=json.loads((cache/'products.json').read_text()) if (cache/'products.json').exists() else {}
        for name,source,extra in [('door_client','door_client.cc',[]),('libwarehouse_door.so','door_plugin.cc',['-shared','-fPIC'])]:
            target=cache/name
            if not target.is_file() or sha(target)!=previous.get(name):
                temporary=cache/('compile-'+uuid.uuid4().hex)
                command=['g++','-std=c++17','-O2','-Wno-deprecated-declarations']+extra+cflags+[str(root/source),'-o',str(temporary)]+libs
                commands.append(command)
                try:
                    result=subprocess.run(command,text=True,capture_output=True,timeout=90)
                    if result.returncode: raise RuntimeError(result.stdout+'\n'+result.stderr)
                    os.replace(str(temporary),str(target))
                finally:
                    if temporary.exists(): temporary.unlink()
            products[name]=sha(target)
            link=build/name
            if link.exists() or link.is_symlink(): link.unlink()
            link.symlink_to(target)
        temporary=cache/('products-'+uuid.uuid4().hex+'.json');temporary.write_text(json.dumps(products)+'\n');temporary.replace(cache/'products.json')
    world=ET.parse(str(root/'world.sdf'))
    if world.find("world/model[@name='warehouse_bot']/link[@name='chassis']/sensor[@type='ray']") is None:
        raise RuntimeError('Real CPU ray sensor missing')
    # The catalog loads the existing, reviewable source world. The controller
    # sends the fingerprinted force cap through an acknowledged native command.
    parsed=subprocess.run(['gz','sdf','-p',str(root/'world.sdf')],text=True,capture_output=True,timeout=12)
    if parsed.returncode or ET.fromstring(parsed.stdout).find("world/model[@name='warehouse_door']/joint[@name='slide_joint']") is None:
        raise RuntimeError('Actual converted world/joint is invalid: '+parsed.stderr)
    checked=subprocess.run(['gz','sdf','-k',str(root/'world.sdf')],text=True,capture_output=True,timeout=12)
    if 'Check complete' not in checked.stdout+checked.stderr:
        raise RuntimeError('gz sdf -k success marker missing: '+checked.stdout+checked.stderr)
    for key_name,setting in (('/use_sim_time','true'),('/warehouse/tf_prefix',"''")):
        subprocess.run(['rosparam','set',key_name,setting],check=True,timeout=6)
    record={'parameters':value,'parameters_sha256':sha(root/'parameters.json'),'source_sha256':sources,
            'products':products,'cache_key':key,'cache':str(cache),'build_identity':build_identity,
            'ros_plugins':ros_plugins,'world_sha256':sha(root/'world.sdf'),
            'server_identity':{'path':str(server.resolve()),'sha256':sha(server),'versioned_samefile':str(exact)},
            'gazebo_version':version,'compiler_commands':commands}
    temporary=build/('warehouse-'+uuid.uuid4().hex+'.json');temporary.write_text(json.dumps(record,indent=2)+'\n');temporary.replace(build/'warehouse.json')
    print(json.dumps({'prepared':True,'cache':str(cache),'products':products}))


if __name__=='__main__': main()


