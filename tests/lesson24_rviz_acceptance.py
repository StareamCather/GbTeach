#!/usr/bin/env python3
"""Actual RViz window capture, PID reopen and cleanup; inspect saved images separately.

Run on the attached X11 desktop after sourcing the existing environment. Numeric
TF acceptance lives in lesson24_acceptance.py; this script does not infer render
success from external TF messages or a live process.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.ui_probe import terminal_window_ids
from PyQt5 import QtWidgets,QtGui


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tf-prefix',choices=('','demo'),default='')
    parser.add_argument('--screenshot',type=Path,default=ROOT/'docs/images/lesson24-rviz.png')
    parser.add_argument('--reopen-screenshot',type=Path,default=ROOT/'.deploy/lesson24-rviz-reopened.png')
    parser.add_argument('--report',type=Path,default=ROOT/'.deploy/lesson24-rviz.json')
    args=parser.parse_args()
    started=time.monotonic()
    app=QtWidgets.QApplication([])
    lessons,errors=load_catalog(ROOT)
    assert not errors,errors
    lesson=next(x for x in lessons if x.id=='24')
    storage=Storage(tempfile.mkdtemp(prefix='lesson24-rviz-',dir=str(ROOT/'.deploy')))
    session=LessonSession(ROOT,lesson,storage=storage)
    (session.workspace/'arm_settings.json').write_text(json.dumps({'joint_state_rate_hz':20,'tf_prefix':args.tf_prefix})+'\n',encoding='utf-8')
    source_sha256={p.relative_to(lesson.folder).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(lesson.folder.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
    result={'source_sha256':source_sha256,'passed':False,'token':session.token,'storage':str(storage.base),'captures':[],
            'tf_prefix':args.tf_prefix,'render_check_automated':False,
            'note':'Passed checks cover owned windows/PIDs and cleanup; inspect RobotModel and TF in both real images.'}
    wrappers=[]
    def process(pid):
        try:
            p=Path('/proc',str(pid));s=(p/'stat').read_text().rsplit(') ',1)[1].split()
            return {'pid':pid,'state':s[0],'pgid':os.getpgid(pid),'sid':os.getsid(pid),'start_ticks':s[19],
                    'argv':(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace'),
                    'token_present':('GAZEBOTEACH_SESSION_TOKEN='+session.token).encode() in (p/'environ').read_bytes().split(b'\0')}
        except (OSError,IndexError,ValueError):return None
    def manager(operation):
        if operation!='status':
            output=session.session_dir/('operator-rviz-'+operation+'.json')
            session.command('python3 rviz_manager.py '+operation+' > '+shlex.quote(str(output)),timeout=15,label='Owned RViz '+operation)
            return json.loads(output.read_text(encoding='utf-8').strip().splitlines()[-1])
        c=subprocess.run(['/usr/bin/python3','rviz_manager.py',operation],cwd=str(session.workspace),env=session.env,capture_output=True,text=True,timeout=15)
        if c.returncode:raise RuntimeError(c.stdout+c.stderr)
        return json.loads(c.stdout.strip().splitlines()[-1])
    def capture(pid,name):
        deadline=time.monotonic()+25
        identity=None
        while time.monotonic()<deadline:
            clients=subprocess.check_output(['xprop','-root','_NET_CLIENT_LIST'],text=True,timeout=3)
            for xid in re.findall(r'0x[0-9a-fA-F]+',clients):
                prop=subprocess.run(['xprop','-id',xid,'_NET_WM_PID'],capture_output=True,text=True,timeout=3).stdout
                if re.search(r'=\s*'+str(pid)+r'\s*$',prop):
                    info=subprocess.run(['xwininfo','-id',xid],capture_output=True,text=True,timeout=3).stdout
                    width=re.search(r'Width:\s*(\d+)',info);height=re.search(r'Height:\s*(\d+)',info)
                    if width and height and int(width.group(1))>=800 and int(height.group(1))>=600:
                        identity=int(xid,16);break
            if identity:break
            app.processEvents();time.sleep(.1)
        if identity is None:raise RuntimeError('No actual RViz window belonging to PID '+str(pid))
        window=QtGui.QWindow.fromWinId(identity);wrappers.append(window)
        window.showNormal();window.raise_();window.requestActivate()
        until=time.monotonic()+2
        while time.monotonic()<until:app.processEvents();time.sleep(.03)
        output=Path(name).resolve()
        output.parent.mkdir(parents=True,exist_ok=True)
        pixmap=app.primaryScreen().grabWindow(identity)
        assert not pixmap.isNull() and pixmap.width()>400 and pixmap.height()>300
        assert pixmap.save(str(output))
        return {'pid':pid,'xid':hex(identity),'screenshot':str(output),'bytes':output.stat().st_size,'width':pixmap.width(),'height':pixmap.height(),'render_review':'pending root image inspection'}
    try:
        session.start(gui=True,terminals=True)
        result.update(workspace=str(session.workspace),session_dir=str(session.session_dir))
        for xid in terminal_window_ids(session):
            window=QtGui.QWindow.fromWinId(xid);wrappers.append(window);window.showMinimized()
        app.processEvents()
        first=manager('status');assert first['running']
        result['initial_status']=first
        result['captures'].append(capture(first['identity']['pid'],args.screenshot))
        result['initial_process_after_render']=process(first['identity']['pid'])
        result['status_after_render']=manager('status')
        result['reopen_result']=manager('reopen')
        second=manager('status');assert second['running'] and second['identity']['pid']!=first['identity']['pid']
        deadline=time.monotonic()+3
        while first['identity']['pid'] in live_token_processes(session.token) and time.monotonic()<deadline:time.sleep(.03)
        result['old_process_after_wait']=process(first['identity']['pid'])
        result['reopened_status']=second
        result['captures'].append(capture(second['identity']['pid'],args.reopen_screenshot))
        assert first['identity']['pid'] not in live_token_processes(session.token),'Old owned RViz PID remains after reopen'
        assert session.server.poll() is None
        result['other_components_survive_reopen']=session.health()
        result['passed']=True
    except Exception as error:
        result['error']=type(error).__name__+': '+str(error)
    finally:
        session.stop(reason='L24 actual RViz prototype complete')
        result['leftovers']=live_token_processes(session.token)
        if result['leftovers']:result['passed']=False
        for src,dst in [('lesson24-rviz.log','lesson24-rviz-final.log'),('lesson24-rendered.rviz','lesson24-rviz-final.rviz')]:
            source=session.session_dir/src
            if source.is_file():shutil.copy2(source,ROOT/'.deploy'/dst)
        shutil.copy2(session.log_path,ROOT/'.deploy/lesson24-rviz-final-runtime.log')
        result['elapsed_seconds']=time.monotonic()-started
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':sys.exit(main())
