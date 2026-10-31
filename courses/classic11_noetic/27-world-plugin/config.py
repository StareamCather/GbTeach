"""Finite factory inputs and fingerprints of the personal offline SDK build."""
import hashlib
import json
import math
from pathlib import Path

SOURCES = ('world_plugin.cc','observer.cc','config.py','prepare.py','lab.py','verify.py')
WORLD = 'lesson27'
GROUND_TOP = .1
BLOCK_SIDE = .16
GRAVITY = 9.8


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value)!={'count','drop_height'} or type(value['count']) is not int or not 1<=value['count']<=5:
        raise ValueError('Use exactly count (integer 1..5) and drop_height (.6..2 metres)')
    h=value['drop_height']
    if type(h) not in (int,float) or not math.isfinite(h) or not .6<=h<=2:
        raise ValueError('drop_height must be finite within [.6,2] metres absolute centre z')
    return {'count':value['count'],'drop_height':float(h)}


def prepared(workspace=None):
    workspace=Path(workspace or Path.cwd()).resolve()
    record=json.loads((workspace/'build/world_lab.json').read_text(encoding='utf-8'))
    value=parameters(workspace/'parameters.json')
    if value!=record['parameters'] or sha(workspace/'parameters.json')!=record['config_sha256']:
        raise RuntimeError('Saved parameters changed; reload/restart to apply them')
    if sha(workspace/'world.sdf')!=record['world_sha256']:
        raise RuntimeError('World/SDF changed; reload/restart required')
    for name,expected in record['source_sha256'].items():
        if sha(workspace/name)!=expected:raise RuntimeError('Source changed; reload/restart required: '+name)
    for name,field in [('build/lib/libgazeboteach_l27_world.so','library_sha256'),('build/world_observer','observer_sha256')]:
        if sha(workspace/name)!=record[field]:raise RuntimeError('Compiled product differs from prepared metadata: '+name)
    return value,record
