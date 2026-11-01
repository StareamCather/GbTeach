#!/usr/bin/env python3
"""Offline Classic SDK build; personal C++ edits participate in the cache key."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import uuid
import xml.etree.ElementTree as ET
from config import INERTIA, SOURCES, parameters, sha


def main():
    workspace = Path.cwd().resolve()
    value = parameters()
    build = workspace/'build'
    if workspace not in build.resolve().parents:
        raise ValueError('Build path leaves the personal workspace')
    (build/'lib').mkdir(parents=True, exist_ok=True)
    world = workspace/'world.sdf'
    xml = ET.parse(str(world))
    model = xml.find("world/model[@name='rotor_lab']")
    if model is None or xml.find('world').get('name') != 'lesson26':
        raise ValueError('Keep the lesson26/rotor_lab experiment identities')
    inertia = float(model.find("link[@name='rotor']/inertial/inertia/izz").text)
    if abs(inertia-INERTIA) > 1e-12:
        raise ValueError('This comparison requires actual Izz=.015; keep the original rotor inertia')
    plugin = model.find("plugin[@name='rotor_model']")
    if plugin is None or plugin.get('filename') != 'libgazeboteach_l26_model.so':
        raise ValueError('ModelPlugin filename/name is missing or changed')
    plugin.find('drive_torque').text = format(value['drive_torque'], '.17g')
    plugin.find('linear_drag').text = format(value['drag'], '.17g')
    plugin.find('config_sha256').text = sha(workspace/'parameters.json')
    xml.write(str(world), encoding='utf-8', xml_declaration=True)
    version = subprocess.check_output(['pkg-config', '--modversion', 'gazebo'], text=True, timeout=5).strip()
    if version != '11.15.1':
        raise RuntimeError('Use the actual Gazebo Classic 11 SDK')
    flag_text = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gazebo'], text=True, timeout=5).strip()
    compiler = subprocess.check_output(['g++', '--version'], text=True, timeout=5).splitlines()[0]
    source_hashes = {name: sha(workspace/name) for name in SOURCES}
    key = hashlib.sha256((source_hashes['model_plugin.cc']+source_hashes['observer.cc']+version+flag_text+compiler).encode()).hexdigest()
    cache = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache')))/'gazeboteach/model-plugin'/key[:24]
    cache.mkdir(parents=True, exist_ok=True)
    commands = []
    for source, name, extra, target in [
        ('model_plugin.cc', 'libgazeboteach_l26_model.so', ['-shared', '-fPIC', '-DGZTEACH_SOURCE_SHA="'+source_hashes['model_plugin.cc']+'"'], build/'lib/libgazeboteach_l26_model.so'),
        ('observer.cc', 'model_observer', [], build/'model_observer')]:
        output = cache/name
        if not output.is_file():
            temporary = cache/('compile-'+uuid.uuid4().hex)
            command = ['g++', '-std=c++17', '-O2', '-Wno-deprecated-declarations']+extra+[str(workspace/source), '-o', str(temporary)]+shlex.split(flag_text)+["-ldl"]
            commands.append(command)
            try:
                result = subprocess.run(command, text=True, capture_output=True, timeout=65)
                if result.returncode:
                    raise RuntimeError('Actual compiler failed:\n'+result.stdout+'\n'+result.stderr)
                os.replace(str(temporary), str(output))
            finally:
                if temporary.exists(): temporary.unlink()
        if target.exists() or target.is_symlink(): target.unlink()
        target.symlink_to(output)
    record = {'parameters': value, 'config_sha256': sha(workspace/'parameters.json'),
              'source_sha256': source_hashes, 'world_sha256': sha(world), 'gazebo_version': version,
              'compiler': compiler, 'compiler_commands': commands, 'cache_key': key,
              'library_sha256': sha(build/'lib/libgazeboteach_l26_model.so'),
              'observer_sha256': sha(build/'model_observer'), 'actual_inertia_kg_m2': inertia}
    (build/'model_lab.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'prepared': True, 'parameters': value, 'cache': str(cache), 'library_sha256': record['library_sha256']}))


if __name__ == '__main__':
    main()
