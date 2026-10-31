"""The diagnosis is based on input/native evidence, never a case filename."""
import hashlib
import json
from pathlib import Path

WORLD = 'lesson35'
DT_NS = 1000000
CASES = ('malformed-sdf', 'missing-plugin', 'zero-gravity', 'missing-collision')
SOURCES = ('observer.cc', 'force_plugin.cc', 'config.py', 'prepare.py', 'lab.py', 'clinic.py', 'verify.py')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepared(workspace=None):
    root = Path(workspace or Path.cwd()).resolve()
    record = json.loads((root/'build/clinic_lab.json').read_text(encoding='utf-8'))
    for name, expected in record['source_sha256'].items():
        if sha(root/name) != expected:
            raise RuntimeError('Source changed; reload before running: '+name)
    for name, key in [('build/clinic_observer', 'observer_sha256'),
                      ('build/libfault_force.so', 'plugin_sha256'),
                      ('world.sdf', 'world_sha256')]:
        if sha(root/name) != record[key]:
            raise RuntimeError('Prepared native product changed; reload: '+name)
    return record
