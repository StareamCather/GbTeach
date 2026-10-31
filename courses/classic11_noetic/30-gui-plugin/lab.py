#!/usr/bin/env python3
"""Real Qt slots, native world feedback and bounded control completion."""
import argparse
import json
import time
from pathlib import Path
import subprocess
from config import prepared,parameters

def require(value,message):
    if not value:raise RuntimeError(message)
def decode(text):
    rows=[]
    for line in text.splitlines():
        try:value=json.loads(line)
        except ValueError:continue
        if isinstance(value,dict):rows.append(value)
    return rows
def ns(stamp):return int(stamp.get('sec',0))*1000000000+int(stamp.get('nsec',0))
class Lab:
    def __init__(self):
        self.settings,self.record=prepared();self.workspace=Path.cwd().resolve();self.binary=self.workspace/'build/panel_observer'
    def client(self,mode,*args,timeout=8):
        p=subprocess.run([str(self.binary),mode]+list(args),text=True,capture_output=True,timeout=timeout)
        require(p.returncode==0,'Actual SDK client failed: '+p.stdout[-1500:]+' '+p.stderr[-900:])
        rows=decode(p.stdout);require(rows,'Actual SDK returned no result');return rows[-1]
    def stats(self):return self.client('stats')['data']
    def entity(self):return self.client('entity','--name','falling_cube')['data']
    def snapshot(self):return self.client('snapshot')['payload']
    def ready_server(self):
        self.settings,self.record=prepared();stats=self.stats();entity=self.entity()
        require(entity['name']=='falling_cube','Wrong actual model')
        return {'ready':True,'scope':'native server; GUI presence not implied','world_stats':stats,'native_cube':entity}
    def ready_panel(self):
        self.settings,self.record=prepared();deadline=time.monotonic()+10
        while True:
            s=self.snapshot()
            require(s['world']=='lesson30','Wrong actual panel world')
            require(s['source_sha256']==self.record['source_sha256']['gui_plugin.cc'] and s['config_sha256']==self.record['config_sha256'],'Actually loaded source/config differs')
            require(s['step_count']==self.settings['step_count'] and s['display_decimals']==self.settings['display_decimals'],'Actually loaded panel inputs differ')
            require(s['load_count']==1,'Panel Load did not run once')
            require(not s['last_error'],'Panel reports: '+s['last_error'])
            if s['stats_available'] and s['timer_ticks']>1 and all(b['width']>20 and b['height']>15 for b in s['buttons'].values()):break
            require(time.monotonic()<deadline,'Actual GUI panel/statistics not ready')
            time.sleep(.05)
        return {'ready':True,'state':s,'native_world':self.stats(),'library_sha256':self.record['library_sha256']}
    def click(self,action):
        self.ready_panel();before=self.snapshot();require(not before['busy'],'A panel control is already pending')
        require(before['buttons'][action]['enabled'],'Panel button is disabled: '+action)
        accepted=self.client('command','--action',action)['payload'];require(accepted['accepted'],'Panel request rejected')
        # One queued QPushButton.click request. Never replay it to meet a target.
        deadline=time.monotonic()+9
        while True:
            after=self.snapshot();require(not after['last_error'],'Panel error: '+after['last_error'])
            if after['completed_controls']>=before['completed_controls']+1 and not after['busy']:break
            require(time.monotonic()<deadline,'Panel control did not complete; request was NOT replayed')
            time.sleep(.04)
        require(after['published_controls']==before['published_controls']+1,'Panel did not send exactly one control')
        require(after['click_counts'][action]==before['click_counts'][action]+1,'Actual Qt slot invocation differs')
        require(after['last_completed_action']==action,'Wrong completed actual Qt action')
        return {'passed':True,'action':action,'before':before,'accepted':accepted,'after':after,'native_world':self.stats(),
                'interaction':'Course automation queues QPushButton.click on the GUI thread; independent acceptance separately uses real X11 mouse events.'}
    def pause(self,value):
        now=self.stats()
        if bool(now.get('paused',False))!=value:self.click('pause' if value else 'resume')
        after=self.stats();require(bool(after.get('paused',False))==value,'Actual pause state differs');return after
    def demo(self):
        self.ready_panel();entry=self.stats();pause=bool(entry.get('paused',False));result={}
        try:
            self.pause(True);before=self.stats();cube_before=self.entity();step=self.click('step');after=self.stats();cube_after=self.entity()
            require(after['paused'] and ns(after['sim_time'])-ns(before['sim_time'])==self.settings['step_count']*1000000,'Actual native step clock differs')
            require(int(after['iterations'])-int(before['iterations'])==self.settings['step_count'],'Actual native physics tick count differs')
            require(cube_after['pose']['position']['z']<=cube_before['pose']['position']['z']+1e-7,'Cube moved upward unexpectedly')
            resumed=self.click('resume');running=self.stats();target=ns(running['sim_time'])+150000000;deadline=time.monotonic()+5
            while ns(self.stats()['sim_time'])<target:
                require(time.monotonic()<deadline,'Native world did not advance after actual Resume')
            stopped=self.click('pause');final=self.stats()
            require(final['paused'] and ns(final['sim_time'])>ns(after['sim_time']),'Actual Resume/Pause did not change world state')
            panel=self.snapshot();require(panel['label_state']=='paused','Panel did not display actual paused status')
            result={'passed':True,'step':step,'resume':resumed,'pause':stopped,'native_before':before,'native_after_step':after,
                    'native_cube_before':cube_before,'native_cube_after_step':cube_after,'native_after_motion':final,'panel':panel,
                    'policy':'Restores entry pause only. Gravity motion, simulation time, message history and button counters are real and are not rewound.'}
        finally:result['final_world']=self.pause(pause)
        return result
    def verify(self):return self.demo()
    def watch(self,seconds):
        require(.1<=seconds<=8,'Watch .1..8 seconds')
        value=self.client('watch','--seconds',str(seconds),timeout=seconds+8)
        return {'last_wire_record':value,'panel':self.snapshot()}
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('ready-server','ready-panel','ready','pause','resume','step','demo','verify','watch','predict'))
    p.add_argument('--seconds',type=float,default=2);args=p.parse_args()
    if args.action=='predict':
        s=parameters();t=s['step_count']*.001;result={'parameters':s,'step_seconds':t,'free_fall_displacement_from_rest_m':.5*9.8*t*t,
            'note':'Only a fresh cube above contact and initially at rest follows this displacement. GUI formatting never changes physical time.'}
    else:
        lab=Lab()
        if args.action in ('ready','ready-panel'):result=lab.ready_panel()
        elif args.action=='ready-server':result=lab.ready_server()
        elif args.action in ('pause','resume','step'):result=lab.click(args.action)
        elif args.action=='watch':result=lab.watch(args.seconds)
        else:result=getattr(lab,args.action)()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
if __name__=='__main__':
    try:main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
