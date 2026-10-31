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
from config import SOURCES, sha


def output(command, timeout=20):
    return subprocess.check_output(command, text=True, timeout=timeout).strip()


def sdk_inputs(root, cflags, libs):
    # Ask the actual compiler for its transitive header set, including Boost,
    # protobuf, ignition and C++ headers. A version string alone is insufficient.
    inputs = {}
    for source in ('observer.cc', 'force_plugin.cc'):
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
    root=Path.cwd().resolve(); build=root/'build'; build.mkdir(exist_ok=True)
    if root not in build.resolve().parents: raise ValueError('Build leaves learner workspace')
    version=output(['pkg-config','--modversion','gazebo'])
    if version!='11.15.1': raise RuntimeError('IO guard supports Classic11.15.1 only')
    server=Path(shutil.which('gzserver') or '/missing')
    exact=Path('/usr/bin/gzserver-11.15.1')
    if not server.is_file() or not exact.is_file() or not os.path.samefile(str(server),str(exact)):
        raise RuntimeError('gzserver must be the installed Classic11.15.1 executable')
    native=Path(shutil.which('gz') or '/missing').resolve()
    if not native.is_file(): raise RuntimeError('Native gz sdf tool is unavailable')
    cflags=shlex.split(output(['pkg-config','--cflags','gazebo']))
    libs=shlex.split(output(['pkg-config','--libs','gazebo']))+['-ldl']
    sources={name:sha(root/name) for name in SOURCES}
    identity={'sources':sources,'sdk_inputs':sdk_inputs(root,cflags,libs),
              'gazebo_version':version,'compiler':output(['g++','--version']).splitlines()[0],
              'cflags':cflags,'libs':libs,'options':['-std=c++17','-O2','-Wno-deprecated-declarations']}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/fault-clinic'/key
    cache.mkdir(parents=True,exist_ok=True); commands=[]
    products={'clinic_observer':('observer.cc',[]), 'libfault_force.so':('force_plugin.cc',['-shared','-fPIC'])}
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        hashes=json.loads((cache/'products.json').read_text()) if (cache/'products.json').is_file() else {}
        for name,(source,extra) in products.items():
            product=cache/name
            if not product.is_file() or hashes.get(name)!=sha(product):
                temporary=cache/('compile-'+uuid.uuid4().hex)
                command=['g++']+identity['options']+extra+cflags+[str(root/source),'-o',str(temporary)]+libs
                commands.append(command)
                try:
                    result=subprocess.run(command,text=True,capture_output=True,timeout=90)
                    if result.returncode: raise RuntimeError('Actual compiler failed:\n'+result.stdout+'\n'+result.stderr)
                    os.replace(str(temporary),str(product))
                finally:
                    if temporary.exists(): temporary.unlink()
            hashes[name]=sha(product)
            target=build/name
            if target.exists() or target.is_symlink(): target.unlink()
            target.symlink_to(product)
        temporary=cache/('products-'+uuid.uuid4().hex)
        temporary.write_text(json.dumps(hashes,sort_keys=True)+'\n')
        temporary.replace(cache/'products.json')
    record={'source_sha256':sources,'world_sha256':sha(root/'world.sdf'),'cache_key':key,'cache':str(cache),
            'observer_sha256':hashes['clinic_observer'],'plugin_sha256':hashes['libfault_force.so'],
            'gazebo_version':version,'build_identity':identity,'compiler_commands':commands,
            'server_identity':{'path':str(server.resolve()),'sha256':sha(server),'samefile_versioned':str(exact)},
            'parser_identity':{'path':str(native),'sha256':sha(native)}}
    temporary=build/('clinic_lab-'+uuid.uuid4().hex+'.json'); temporary.write_text(json.dumps(record,indent=2)+'\n')
    temporary.replace(build/'clinic_lab.json')
    print(json.dumps({'prepared':True,'cache':str(cache),'cache_key':key,'offline':True}))


if __name__=='__main__': main()


