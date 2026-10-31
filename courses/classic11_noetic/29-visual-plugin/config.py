"""Finite filter parameters and exact fingerprints of the personal SDK build."""
import hashlib
import json
import math
from pathlib import Path

SOURCES=('visual_plugin.cc','observer.cc','config.py','prepare.py','lab.py','verify.py')
WORLD='lesson29'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value)!={'color','visual_scale'}:raise ValueError('Use exactly color and visual_scale')
    if value['color'] not in ('red','blue','green'):raise ValueError('Color must be red, blue or green')
    scale=value['visual_scale']
    if type(scale) not in (int,float) or not math.isfinite(scale) or not .7<=scale<=1.6:
        raise ValueError('visual_scale must be finite within [.7,1.6]')
    return {'color':value['color'],'visual_scale':float(scale)}


def prepared(workspace=None):
    root=Path(workspace or Path.cwd()).resolve()
    record=json.loads((root/'build/visual_lab.json').read_text(encoding='utf-8'))
    value=parameters(root/'parameters.json')
    if value!=record['parameters'] or sha(root/'parameters.json')!=record['config_sha256']:
        raise RuntimeError('Saved parameters changed; reload/restart before applying them')
    if sha(root/'world.sdf')!=record['world_sha256']:raise RuntimeError('World changed; reload/restart required')
    for name,expected in record['source_sha256'].items():
        if sha(root/name)!=expected:raise RuntimeError('Source changed; reload/restart required: '+name)
    for name,key in [('build/lib/libgazeboteach_l29_visual.so','library_sha256'),('build/visual_observer','observer_sha256')]:
        if sha(root/name)!=record[key]:raise RuntimeError('Compiled product differs: '+name)
    return value,record
