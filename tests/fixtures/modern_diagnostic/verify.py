import json
import os
import re
import subprocess
import time


def sample_height(command):
    text = subprocess.check_output(
        [command, 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'],
        text=True, timeout=6)
    # Read native protobuf text, also available in Fortress releases whose CLI
    # predates JSON output. A top-level pose block contains one entity.
    for pose in re.findall(r'^pose\s*\{(.*?)^\}', text, re.MULTILINE | re.DOTALL):
        if not re.search(r'^\s*name:\s*"probe"\s*$', pose, re.MULTILINE):
            continue
        position = re.search(r'\bposition\s*\{([^}]*)\}', pose)
        height = re.search(r'^\s*z:\s*(\S+)', position.group(1), re.MULTILINE) if position else None
        if height:
            return float(height.group(1))
    raise RuntimeError('未收到 probe 模型的原生高度。')


if __name__ == '__main__':
    profile = os.environ.get('GAZEBOTEACH_PROFILE', 'harmonic-jazzy')
    command, partition_key = {
        'harmonic-jazzy': ('gz', 'GZ_PARTITION'),
        'fortress-humble': ('ign', 'IGN_PARTITION'),
    }[profile]
    time.sleep(1)
    z = sample_height(command)
    assert abs(z - .1) < .04, {'expected_z': .1, 'actual_z': z}
    print(json.dumps({'passed': True, 'probe_z': z, 'partition': os.environ[partition_key]}), flush=True)
