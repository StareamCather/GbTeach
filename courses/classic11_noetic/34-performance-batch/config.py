"""Bounded, deterministic Classic11 batch inputs; no Python RNG is used."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

WORLD = 'lesson34'
DT_NS = 1000000
SOURCES = ('observer.cc', 'config.py', 'prepare.py', 'lab.py', 'batch.py', 'verify.py')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameters(path=None):
    value = json.loads(Path(path or 'parameters.json').read_text(encoding='utf-8'))
    if set(value) != {'counts', 'step_sizes_s', 'repeats', 'seed'}:
        raise ValueError('Use exactly counts, step_sizes_s, repeats and seed')
    for key, allowed in [('counts', (8, 24)), ('step_sizes_s', (.001, .002))]:
        rows = value[key]
        if not isinstance(rows, list) or not rows or any(type(x) not in (int, float) or x not in allowed for x in rows) or len(set(rows)) != len(rows):
            raise ValueError(key + ' must be a nonempty unique subset of ' + str(allowed))
    if any(type(x) is not int for x in value['counts']):
        raise ValueError('counts must contain integers')
    if type(value['repeats']) is not int or not 1 <= value['repeats'] <= 3:
        raise ValueError('repeats must be an integer in [1,3]')
    if type(value['seed']) is not int or not 0 <= value['seed'] <= 2147483647:
        raise ValueError('seed must be an integer in [0,2147483647]')
    return value


def world_bytes(template, count, dt, rate):
    """Copy the offline preview; replace only its grid and physics settings."""
    xml = ET.parse(str(template)); world = xml.find('world')
    if world is None or world.get('name') != WORLD or list(xml.iter('plugin')) or list(xml.iter('uri')):
        raise ValueError('Keep the self-contained lesson34 world without plugins/URIs')
    prototype = world.find("model[@name='cube_000']")
    if prototype is None or world.find("model[@name='floor']") is None:
        raise ValueError('Keep floor and cube_000 in the preview')
    serialized = ET.tostring(prototype, encoding='unicode')
    for model in list(world.findall('model')):
        if model.get('name', '').startswith('cube_'): world.remove(model)
    for index in range(count):
        model = ET.fromstring(serialized); model.set('name', 'cube_%03d' % index)
        x = (index % 6 - 2.5) * .5; y = (index // 6 - 1.5) * .5
        model.find('pose').text = '%.17g %.17g 2 0 0 0' % (x, y)
        world.append(model)
    physics = world.find('physics')
    physics.find('max_step_size').text = format(dt, '.17g')
    physics.find('real_time_update_rate').text = str(rate)
    return ET.tostring(xml.getroot(), encoding='utf-8', xml_declaration=True)


def prepared(workspace=None):
    root = Path(workspace or Path.cwd()).resolve(); value = parameters(root/'parameters.json')
    record = json.loads((root/'build/batch_lab.json').read_text(encoding='utf-8'))
    if value != record['parameters'] or sha(root/'parameters.json') != record['config_sha256']:
        raise RuntimeError('Parameters changed; reload before running')
    for name, expected in record['source_sha256'].items():
        if sha(root/name) != expected: raise RuntimeError('Source changed; reload: ' + name)
    for name, key in [('world.sdf', 'world_sha256'), ('build/batch_observer', 'observer_sha256')]:
        if sha(root/name) != record[key]: raise RuntimeError('Prepared input/product changed; reload: ' + name)
    return value, record
