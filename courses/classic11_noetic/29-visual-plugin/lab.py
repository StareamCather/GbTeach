#!/usr/bin/env python3
"""Render-thread requests, actual user-camera frames and native physics checks."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from config import parameters,prepared

def require(value,message):
    if not value:raise RuntimeError(message)

def decode(text):
    rows=[]
    for line in text.splitlines():
        try:value=json.loads(line)
        except ValueError:continue
        if isinstance(value,dict):rows.append(value)
    return rows

def pixels(path):
    """Classify an ACTUAL camera PNG; no predicted image is constructed."""
    from PyQt5.QtGui import QImage
    image=QImage(str(path));require(not image.isNull(),'Actual camera PNG cannot be decoded')
    image=image.convertToFormat(QImage.Format_RGB888)
    raw=image.bits().asstring(image.byteCount());width,height=image.width(),image.height();stride=image.bytesPerLine()
    values={'red':0,'blue':0,'green':0,'yellow':0}
    for y in range(height):
        line=raw[y*stride:y*stride+3*width]
        for r,g,b in zip(line[0::3],line[1::3],line[2::3]):
            if r>150 and r>2*g and r>2*b:values['red']+=1
            elif b>150 and b>2*r and b>2*g:values['blue']+=1
            elif g>150 and g>2*r and g>2*b:values['green']+=1
            elif r>150 and g>150 and b<80:values['yellow']+=1
    return {'width':width,'height':height,'counts':values,'sha256':hashlib.sha256(Path(path).read_bytes()).hexdigest()}

class Lab:
    def __init__(self):
        self.settings,self.record=prepared();self.workspace=Path.cwd().resolve();self.binary=self.workspace/'build/visual_observer'
    def client(self,mode,*args,timeout=8):
        p=subprocess.run([str(self.binary),mode]+list(args),text=True,capture_output=True,timeout=timeout)
        rows=decode(p.stdout)
        require(p.returncode==0,'Actual native client failed: '+p.stdout[-1400:]+' '+p.stderr[-700:])
        require(rows,'Actual native client returned no message')
        return rows[-1]
    def stats(self):return self.client('stats')['data']
    def entity(self,name='beacon'):return self.client('entity','--name',name)['data']
    def snapshot(self):return self.client('snapshot')['payload']
    def ready_server(self):
        self.settings,self.record=prepared();beacon=self.entity();reference=self.entity('reference')
        require(beacon['name']=='beacon' and reference['name']=='reference','Wrong private world entities')
        return {'ready':True,'scope':'server only; does not certify VisualPlugin or rendered pixels',
                'native_beacon':beacon,'native_reference':reference,'world_stats':self.stats()}
    def ready_client(self):
        self.settings,self.record=prepared();deadline=time.monotonic()+18
        last=None
        while True:
            try:
                s=self.snapshot()
                require(s['source_sha256']==self.record['source_sha256']['visual_plugin.cc'] and s['config_sha256']==self.record['config_sha256'],
                        'Actually loaded client plugin differs; reload project')
                require(s['configured_color']==self.settings['color'] and abs(s['configured_visual_scale']-self.settings['visual_scale'])<1e-10,'Stale actual client parameters')
                if s['appearance_applied'] and s['camera_initialized'] and s['render_count']>2:
                    require(s['load_count']==1 and s['init_count']>=1,'VisualPlugin lifecycle not observed')
                    require(not s['last_error'],'Actual render error: '+s['last_error'])
                    return {'ready':True,'state':s,'scope':'actual client appearance and camera readback; pixels verified separately'}
            except RuntimeError as error:last=str(error)
            require(time.monotonic()<deadline,'A real gzclient is required; VisualPlugin is client-only. '+str(last))
            time.sleep(.05)
    def ready(self):return {'server':self.ready_server(),'client':self.ready_client()}
    def command(self,action,**values):
        arguments=['--action',action]
        for key,value in values.items():arguments+=['--'+key,str(value)]
        receipt=self.client('command',*arguments)['payload'];require(receipt['accepted'],'Render request rejected')
        sequence=receipt['command_seq'];deadline=time.monotonic()+8
        while True:
            actual=self.snapshot();require(not actual['last_error'],'Actual render error: '+actual['last_error'])
            done=actual['captured_command_seq']>=sequence if action=='capture' else actual['applied_command_seq']>=sequence
            if done:return {'accepted_receipt':receipt,'actual_after':actual}
            require(time.monotonic()<deadline,'Render command not completed; request not replayed')
            time.sleep(.03)
    def pause(self,value):
        if bool(self.stats().get('paused',False))!=value:self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+5
        while True:
            actual=self.stats()
            if bool(actual.get('paused',False))==value:return actual
            require(time.monotonic()<deadline,'Native pause not completed; request not replayed')
    def capture(self,path=None):
        path=Path(path or self.workspace/'build'/('actual-render-'+uuid.uuid4().hex+'.png')).expanduser().resolve()
        path.parent.mkdir(parents=True,exist_ok=True)
        require(not path.exists(),'Use a new capture filename to preserve earlier evidence')
        result=self.command('capture',path=str(path));require(path.is_file(),'Actual user camera did not save the frame')
        result.update(path=str(path),pixel_analysis=pixels(path));return result
    def demo(self):
        self.ready();entry=self.stats();paused=bool(entry.get('paused',False));original=self.snapshot()
        native_before=self.entity();reference_before=self.entity('reference');captures=[];result={}
        try:
            self.pause(True);clock=self.stats();self.command('camera');self.command('scale',scale=1)
            for color in ('red','blue','green'):
                self.command('color',color=color);capture=self.capture();metric=capture['pixel_analysis']['counts']
                require(metric[color]>500 and metric['yellow']>40,'Actual frame lacks the target or independent yellow reference')
                require(all(metric[color]>10*metric[other] for other in ('red','blue','green') if other!=color),'Actual target color was not changed')
                captures.append({'color':color,'scale':1,'capture':capture})
            self.command('scale',scale=1.4);large=self.capture();captures.append({'color':'green','scale':1.4,'capture':large})
            normal=captures[-2]['capture']['pixel_analysis']['counts'];bigger=large['pixel_analysis']['counts']
            ratio=(bigger['green']/bigger['yellow'])/(normal['green']/normal['yellow'])
            require(1.5<ratio<3.5,'Actual relative rendered target area did not grow')
            native_after=self.entity();reference_after=self.entity('reference');after_clock=self.stats()
            require(native_before==native_after and reference_before==reference_after,'Visual operation changed native server model/collision')
            require(clock['sim_time']==after_clock['sim_time'] and after_clock['paused'],'Client rendering unexpectedly advanced paused physics')
            result={'passed':True,'captures':captures,'relative_target_area_ratio':ratio,'native_before':native_before,'native_after':native_after,
                    'native_reference_before':reference_before,'native_reference_after':reference_after,'world_before':clock,'world_after':after_clock,
                    'note':'Actual camera PNG pixels and server native state checked independently. Reframes camera and leaves the fixed lesson view.'}
        finally:
            self.command('color',color=original['color']);self.command('scale',scale=original['visual_scale']);result['final_world']=self.pause(paused)
        return result
    def watch(self,seconds):
        require(math.isfinite(seconds) and .1<=seconds<=10,'Watch .1..10 wall seconds')
        path=self.workspace/'build'/('watch-'+uuid.uuid4().hex+'.jsonl')
        with path.open('w',encoding='utf-8') as stream:
            p=subprocess.run([str(self.binary),'watch','--seconds',str(seconds)],stdout=stream,stderr=subprocess.PIPE,text=True,timeout=seconds+8)
        require(p.returncode==0,'Actual observer failed: '+p.stderr)
        return {'raw_events_file':str(path),'events':decode(path.read_text(encoding='utf-8'))}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('ready-server','ready-client','ready','status','color','scale','reset','camera','capture','demo','verify','watch','pause','resume'))
    p.add_argument('--color',choices=('red','blue','green'),default='red');p.add_argument('--scale',type=float,default=1)
    p.add_argument('--path',type=Path);p.add_argument('--seconds',type=float,default=2);a=p.parse_args();lab=Lab()
    if a.action in ('status','ready'):result=lab.ready()
    elif a.action=='ready-server':result=lab.ready_server()
    elif a.action=='ready-client':result=lab.ready_client()
    elif a.action=='color':result=lab.command('color',color=a.color)
    elif a.action=='scale':result=lab.command('scale',scale=a.scale)
    elif a.action in ('reset','camera'):result=lab.command(a.action)
    elif a.action=='capture':result=lab.capture(a.path)
    elif a.action in ('demo','verify'):result=lab.demo()
    elif a.action=='watch':result=lab.watch(a.seconds)
    else:result={'world_stats':lab.pause(a.action=='pause')}
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))

if __name__=='__main__':
    try:main()
    except Exception as error:print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
