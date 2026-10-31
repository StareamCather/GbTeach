"""Finite native recorder inputs and personal build fingerprints."""
import hashlib
import json
from pathlib import Path

WORLD='lesson32'
DT_NS=1000000
SOURCES=('observer.cc','config.py','prepare.py','lab.py','verify.py')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value)!={'record_steps','encoding'}:raise ValueError('Use exactly record_steps and encoding')
    if type(value['record_steps']) is not int or not 100<=value['record_steps']<=800:
        raise ValueError('record_steps must be an integer in [100,800]')
    if value['encoding'] not in ('zlib','txt','bz2'):raise ValueError('encoding must be zlib, txt or bz2')
    return value


def prepared(workspace=None):
    root=Path(workspace or Path.cwd()).resolve();value=parameters(root/'parameters.json')
    record=json.loads((root/'build/log_lab.json').read_text(encoding='utf-8'))
    if value!=record['parameters'] or sha(root/'parameters.json')!=record['config_sha256']:
        raise RuntimeError('Parameters changed; reload before recording')
    for name,expected in record['source_sha256'].items():
        if sha(root/name)!=expected:raise RuntimeError('Source changed; reload: '+name)
    for name,key in [('world.sdf','world_sha256'),('build/log_observer','observer_sha256')]:
        if sha(root/name)!=record[key]:raise RuntimeError('Prepared input/product changed; reload: '+name)
    return value,record
