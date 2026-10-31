"""Finite learner inputs and fingerprints of the actual compiled private copy."""
import hashlib
import json
import math
from pathlib import Path

SOURCES = ('model_plugin.cc', 'observer.cc', 'config.py', 'prepare.py', 'lab.py', 'verify.py')
INERTIA = .015
WORLD = 'lesson26'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameters(path=None):
    value = json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value) != {'drive_torque', 'drag'} or any(type(x) not in (int, float) or not math.isfinite(x) for x in value.values()):
        raise ValueError('Use exactly finite numeric drive_torque and drag')
    if not (.01 <= abs(value['drive_torque']) <= .09 and .005 <= value['drag'] <= .06):
        raise ValueError('Use |drive_torque| within [.01,.09] Nm and drag within [.005,.06] Nms/rad')
    return {key: float(number) for key, number in value.items()}


def prepared(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    record = json.loads((workspace/'build/model_lab.json').read_text(encoding='utf-8'))
    value = parameters(workspace/'parameters.json')
    if value != record['parameters'] or sha(workspace/'parameters.json') != record['config_sha256']:
        raise RuntimeError('Saved parameters changed; reload/restart to apply them')
    if sha(workspace/'world.sdf') != record['world_sha256']:
        raise RuntimeError('World/SDF changed; reload/restart required')
    for name, expected in record['source_sha256'].items():
        if sha(workspace/name) != expected:
            raise RuntimeError('Source changed; reload/restart to build actual plugin: '+name)
    for name, field in [('build/lib/libgazeboteach_l26_model.so', 'library_sha256'), ('build/model_observer', 'observer_sha256')]:
        if sha(workspace/name) != record[field]:
            raise RuntimeError('Compiled product differs from prepared metadata: '+name)
    return value, record
