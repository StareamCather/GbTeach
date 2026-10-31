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
from config import SOURCES, parameters, sha, world_bytes


def output(command, timeout=20):
    return subprocess.check_output(command, text=True, timeout=timeout).strip()


def sdk_inputs(root, cflags, libs):
    # Ask the actual compiler for its transitive header set, including Boost,
    # protobuf, ignition and C++ headers. A version string alone is insufficient.
    dependency_text = output(['g++', '-std=c++17', '-M', str(root/'observer.cc')] + cflags)
    dependencies = shlex.split(dependency_text.replace('\\\n', ' ').split(':', 1)[1])
    # The source bytes are already in source_sha256. Its personal absolute
    # path must not defeat reuse across distinct learner workspaces.
    inputs = {str(Path(p).resolve()): sha(p) for p in dependencies
              if Path(p).resolve() != (root/'observer.cc').resolve()}
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
    root=Path.cwd().resolve(); value=parameters(); build=root/'build'; build.mkdir(exist_ok=True)
    if root not in build.resolve().parents: raise ValueError('Build path leaves personal workspace')
    world_bytes(root/'world.sdf', 8, .001, 1000)  # validate without modifying GUI input
    version=output(['pkg-config','--modversion','gazebo'])
    if version != '11.15.1': raise RuntimeError('Native IO guard is validated only for Classic11.15.1')
    cflags=shlex.split(output(['pkg-config','--cflags','gazebo']))
    libs=shlex.split(output(['pkg-config','--libs','gazebo'])) + ['-ldl']
    compiler=output(['g++','--version']).splitlines()[0]
    sources={name:sha(root/name) for name in SOURCES}; dependencies=sdk_inputs(root,cflags,libs)
    identity={'sources':sources,'sdk_inputs':dependencies,'gazebo':version,'cflags':cflags,'libs':libs,'compiler':compiler,'options':['-std=c++17','-O2','-Wno-deprecated-declarations']}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/performance-batch'/key
    cache.mkdir(parents=True,exist_ok=True); commands=[]
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX); product=cache/'batch_observer'
        if not product.is_file():
            temporary=cache/('compile-'+uuid.uuid4().hex)
            command=['g++']+identity['options']+cflags+[str(root/'observer.cc'),'-o',str(temporary)]+libs
            commands.append(command)
            try:
                result=subprocess.run(command,text=True,capture_output=True,timeout=90)
                if result.returncode: raise RuntimeError('Actual compiler failed:\n'+result.stdout+'\n'+result.stderr)
                os.replace(str(temporary),str(product))
            finally:
                if temporary.exists(): temporary.unlink()
        target=build/'batch_observer'
        if target.exists() or target.is_symlink(): target.unlink()
        target.symlink_to(product)
    record={'parameters':value,'config_sha256':sha(root/'parameters.json'),'source_sha256':sources,
            'world_sha256':sha(root/'world.sdf'),'gazebo_version':version,'compiler':compiler,
            'compiler_commands':commands,'cache_key':key,'observer_sha256':sha(target),
            'build_identity':identity,'cache':str(cache)}
    (build/'batch_lab.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'prepared':True,'parameters':value,'cache':str(cache),'measurement':'end-to-end wall clock; not pure physics or engine RTF'}))


if __name__=='__main__': main()
