"""Finite SystemPlugin inputs and fingerprints of the personal offline build."""
import hashlib
import json
import math
from pathlib import Path

SOURCES=('system_plugin.cc','observer.cc','config.py','prepare.py','lab.py','verify.py')
WORLD='lesson31'
CUSTOM_TOPIC='/gazebo/lesson31/system_plugin/state'
DT=.001
INITIAL_Z=2.0


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value)!={'gravity_m_s2','record_every_steps'}:raise ValueError('Use exactly gravity_m_s2 and record_every_steps')
    g=value['gravity_m_s2'];n=value['record_every_steps']
    if type(g) not in (int,float) or not math.isfinite(g) or not 1<=g<=20:
        raise ValueError('gravity_m_s2 must be finite within [1,20] m/s^2')
    if type(n) is not int or not 1<=n<=100:raise ValueError('record_every_steps must be an integer within [1,100]')
    return {'gravity_m_s2':float(g),'record_every_steps':n}


def prepared(workspace=None):
    root=Path(workspace or Path.cwd()).resolve()
    record=json.loads((root/'build/system_lab.json').read_text(encoding='utf-8'))
    value=parameters(root/'parameters.json')
    if value!=record['parameters'] or sha(root/'parameters.json')!=record['config_sha256']:
        raise RuntimeError('Saved parameters changed; reload/restart before applying them')
    for name,key in [('world.sdf','world_sha256'),('build/system_plugin.cfg','plugin_config_sha256'),
                     ('build/lib/libgazeboteach_l31_system.so','library_sha256'),('build/system_observer','observer_sha256')]:
        if sha(root/name)!=record[key]:raise RuntimeError('Prepared input/product differs; reload/restart required: '+name)
    for name,expected in record['source_sha256'].items():
        if sha(root/name)!=expected:raise RuntimeError('Source changed; reload/restart required: '+name)
    return value,record
