"""Finite filter parameters and exact fingerprints of the personal SDK build."""
import hashlib
import json
import math
from pathlib import Path

SOURCES=('sensor_plugin.cc','observer.cc','config.py','prepare.py','lab.py','verify.py')
WORLD='lesson28'
NATIVE_TOPIC='/gazebo/lesson28/scanner/link/proximity/scan'
CUSTOM_TOPIC='/gazebo/lesson28/sensor_plugin/state'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value)!={'threshold_m','alpha'}:raise ValueError('Use exactly threshold_m and alpha')
    for key,low,high in [('threshold_m',.2,4),('alpha',.05,1)]:
        v=value[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError(key+' must be finite within ['+str(low)+','+str(high)+']')
    return {k:float(v) for k,v in value.items()}

def prepared(workspace=None):
    root=Path(workspace or Path.cwd()).resolve()
    record=json.loads((root/'build/sensor_lab.json').read_text(encoding='utf-8'))
    value=parameters(root/'parameters.json')
    if value!=record['parameters'] or sha(root/'parameters.json')!=record['config_sha256']:
        raise RuntimeError('Saved parameters changed; reload/restart before applying them')
    if sha(root/'world.sdf')!=record['world_sha256']:raise RuntimeError('World changed; reload/restart required')
    for name,expected in record['source_sha256'].items():
        if sha(root/name)!=expected:raise RuntimeError('Source changed; reload/restart required: '+name)
    for name,key in [('build/lib/libgazeboteach_l28_sensor.so','library_sha256'),('build/sensor_observer','observer_sha256')]:
        if sha(root/name)!=record[key]:raise RuntimeError('Compiled product differs: '+name)
    return value,record
