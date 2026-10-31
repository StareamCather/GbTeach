"""Private Qt SDK build inputs and exact personal source fingerprints."""
import hashlib
import json
from pathlib import Path

SOURCES=('gui_plugin.hh','gui_plugin.cc','observer.cc','config.py','prepare.py','lab.py','verify.py')
WORLD='lesson30'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def parameters(path=None):
    value=json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value)!={'step_count','display_decimals'}:raise ValueError('Use exactly step_count and display_decimals')
    for name,low,high in [('step_count',10,500),('display_decimals',1,6)]:
        if type(value[name]) is not int or not low<=value[name]<=high:raise ValueError(name+' must be an integer within '+str((low,high)))
    return value
def prepared(workspace=None):
    root=Path(workspace or Path.cwd()).resolve();value=parameters(root/'parameters.json')
    record=json.loads((root/'build/gui_lab.json').read_text(encoding='utf-8'))
    if value!=record['parameters'] or sha(root/'parameters.json')!=record['config_sha256']:raise RuntimeError('Saved parameters changed; reload before applying them')
    if sha(root/'world.sdf')!=record['world_sha256']:raise RuntimeError('World changed; reload required')
    for name,expected in record['source_sha256'].items():
        if sha(root/name)!=expected:raise RuntimeError('Source changed; reload required: '+name)
    for name,key in [('build/lib/libgazeboteach_l30_gui.so','library_sha256'),('build/panel_observer','observer_sha256')]:
        if sha(root/name)!=record[key]:raise RuntimeError('Compiled product differs from preparation: '+name)
    return value,record
