"""Compile the personal native plant, with a complete, auditable cache identity."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import uuid


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    version = subprocess.check_output(['pkg-config', '--modversion', 'gazebo'], text=True, timeout=5).strip()
    if version != '11.15.1': raise RuntimeError('Validated against Gazebo Classic 11.15.1; do not silently use another SDK')
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gazebo'], text=True, timeout=5))
    compiler = shutil.which('g++')
    if not compiler: raise RuntimeError('g++ is missing')
    compiler = str(Path(compiler).resolve())
    compiler_version = subprocess.check_output([compiler, '--version'], text=True, timeout=5)
    source = Path('plant.cc').resolve(); source_hash = sha256(source)
    options = ['-shared', '-fPIC', '-std=c++17', '-O2', '-pthread', '-Wno-deprecated-declarations',
               '-DGT_PLANT_SOURCE_SHA="'+source_hash+'"']
    identity = {'source_sha256': source_hash, 'gazebo': version, 'compiler': compiler,
                'compiler_version': compiler_version, 'compile_options': options, 'pkg_config_flags': flags}
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode('utf-8')).hexdigest()
    cache = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache')))/'gazeboteach/quadrotor-plant'/digest
    cache.mkdir(parents=True, exist_ok=True)
    binary = cache/'libgazeboteach_quadrotor.so'; cache_record = cache/'build.json'
    valid = False
    if binary.is_file() and cache_record.is_file():
        record = json.loads(cache_record.read_text(encoding='utf-8'))
        valid = record.get('identity') == identity and record.get('library_sha256') == sha256(binary)
    if not valid:
        temporary = cache/('compile-'+uuid.uuid4().hex+'.so')
        try:
            subprocess.run([compiler]+options+[str(source), '-o', str(temporary)]+flags, check=True, timeout=70)
            os.replace(str(temporary), str(binary))
            cache_record.write_text(json.dumps({'identity': identity, 'library_sha256': sha256(binary)}, indent=2), encoding='utf-8')
        finally:
            if temporary.exists(): temporary.unlink()
    out = Path('build/lib'); out.mkdir(parents=True, exist_ok=True)
    target = out/binary.name
    if target.exists() or target.is_symlink(): target.unlink()
    target.symlink_to(binary)
    metadata = dict(identity, cache_key=digest, library_sha256=sha256(binary), library_path=str(binary))
    (out.parent/'plant-build.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='utf-8')
    print('Real Classic physical plant prepared:', binary, flush=True)


if __name__ == '__main__': main()
