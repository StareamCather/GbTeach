#!/usr/bin/env python3
"""Immutable, small lesson37 configuration and loaded-source fingerprints."""
import hashlib
import json
from pathlib import Path

WORLD='default'
SOURCES=('config.py','prepare.py','arm_plugin.cc','arm_client.cc','mission.py','lab.py','verify.py','world.sdf')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    allowed={'push_duration_s':(4,6,8),'torque_limit_nm':(3,6,9)}
    if set(value)!=set(allowed) or any(type(value[k]) not in (int,float) or value[k] not in v for k,v in allowed.items()):
        raise ValueError('Use push duration 4/6/8 seconds and torque limit 3/6/9 Nm only')
    return value

def prepared(root=None):
    root=Path(root or Path.cwd()).resolve()
    record=json.loads((root/'build/arm_push.json').read_text(encoding='utf-8'))
    value=parameters(root/'parameters.json')
    if value!=record['parameters'] or sha(root/'parameters.json')!=record['parameters_sha256']:
        raise RuntimeError('Parameters changed; reload this course')
    for name in SOURCES:
        if sha(root/name)!=record['source_sha256'][name]:
            raise RuntimeError('Loaded course source differs; reload: '+name)
    for name,digest in record['products'].items():
        if sha(root/'build'/name)!=digest:raise RuntimeError('Native build changed; reload: '+name)
    return value,record

def identity(record):
    return {key:record[key] for key in ('parameters','parameters_sha256','source_sha256','products','ros_plugins')}
