#!/usr/bin/env python3
"""Offline source-addressed SDK build; never download resources."""
import fcntl, hashlib, json, os, shlex, subprocess, uuid
from pathlib import Path
from scene import validate

def main():
 from scene import build
 build()
 validate()
 here=Path.cwd(); sources=[p for p in (here/'native').glob('*.cc')]
 version=subprocess.check_output(['pkg-config','--modversion','gazebo'],text=True,timeout=5).strip()
 if version!='11.15.1':raise RuntimeError('This course requires actual Classic SDK 11.15.1')
 flags=subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True,timeout=5).strip()
 compiler=subprocess.check_output(['g++','--version'],text=True,timeout=5).splitlines()[0]
 key=hashlib.sha256((version+flags+compiler).encode()+b''.join(p.read_bytes() for p in sorted(sources))).hexdigest()
 cache=Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/scenes'/key
 cache.mkdir(parents=True,exist_ok=True);(here/'build/lib').mkdir(parents=True,exist_ok=True)
 with (cache/'compile.lock').open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  for src in sources:
   out=cache/('libgt_'+src.stem+'.so')
   if not out.exists():
    temp=cache/('compile-'+uuid.uuid4().hex)
    cmd=['g++','-std=c++17','-O2','-shared','-fPIC','-Wno-deprecated-declarations',str(src),'-o',str(temp)]+shlex.split(flags)
    try:
     p=subprocess.run(cmd,text=True,capture_output=True,timeout=70)
     if p.returncode:raise RuntimeError('Compiler failed: '+p.stdout+p.stderr)
     os.replace(str(temp),str(out))
    finally:
     if temp.exists():temp.unlink()
   target=here/'build/lib'/out.name
   if target.exists() or target.is_symlink():target.unlink()
   target.symlink_to(out)
 (here/'build/sources.json').write_text(json.dumps({'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},'sdk_version':version,'cache_key':key},indent=2),encoding='utf-8')
 print('Offline scene prepared; SDK',version,'cache',key)
if __name__=='__main__':main()
