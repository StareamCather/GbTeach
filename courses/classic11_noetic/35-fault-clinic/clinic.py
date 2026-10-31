#!/usr/bin/env python3
"""Inject privately, observe real Classic failures, repair a new file, rerun."""
import argparse
import copy
import json
import os
from pathlib import Path
import re
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from config import CASES, WORLD, prepared, sha
from lab import Lab, OwnedTrial, require, write_json


def inject(world, case):
    """Return a damaged copy of supplied bytes; never change the source file."""
    require(case in CASES, 'Unknown educational fault')
    if case=='malformed-sdf':
        require(b'</world>' in world,'Reference lacks world closing tag')
        return world.replace(b'</world>',b'</world_broken>',1)
    xml=ET.fromstring(world,parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    w=xml.find('world'); cube=w.find("model[@name='cube_000']")
    if case=='missing-plugin': cube.find('plugin').set('filename','libl35_missing_force.so')
    elif case=='zero-gravity': w.find('gravity').text='0 0 0'
    elif case=='missing-collision':
        body=cube.find("link[@name='body']"); body.remove(body.find('collision'))
    return ET.tostring(xml,encoding='utf-8',xml_declaration=True)


def plugin_maps(owned):
    """Prove actual mapping against the built ELF's inode and bytes, not basename."""
    expected=Path.cwd().resolve()/'build/libfault_force.so'
    proof={'expected_path':str(expected),'resolved_path':str(expected.resolve()),'expected_sha256':sha(expected),
           'loaded_expected':False,'mappings':[],'processes':owned.owned()}
    for process in proof['processes']:
        if process['pid']!=owned.server.pid: continue
        maps=(Path('/proc')/str(process['pid'])/'maps').read_text()
        (owned.root/'gzserver-proc-maps.txt').write_text(maps,encoding='utf-8')
        for name in sorted(set(line.split(None,5)[-1] for line in maps.splitlines() if len(line.split(None,5))==6)):
            path=Path(name)
            if not path.is_file(): continue
            if os.path.samefile(str(path),str(expected)):
                proof['mappings'].append({'path':str(path),'samefile_expected':True,'sha256':sha(path)})
                proof['loaded_expected']=proof['mappings'][-1]['sha256']==proof['expected_sha256']
    return proof


def trial(input_path, output_dir, checkpoint_hook=None):
    """One immutable input, own server, coherent paused endpoints, raw wire log.

    Optional acceptance hook(phase, Lab, OwnedTrial) runs at each paused endpoint.
    It may independently query the native server; normal course use leaves it None.
    """
    source=Path(input_path).resolve(); root=Path(output_dir).resolve()
    require(not root.exists(), 'Choose a new trial output directory; evidence is never overwritten')
    root.mkdir(parents=True); original=sha(source); record=prepared()
    owned=OwnedTrial(root,source.read_bytes()); entered=False
    result={'input':str(source),'input_sha256':original,'directory':str(root),'completed':False,
            'native_events_file':str(root/'native-events.jsonl'),'provenance':record}
    try:
        command=[record['parser_identity']['path'],'sdf','-k',str(owned.world)]
        parsed=subprocess.run(command,cwd=str(root),env=owned.env,text=True,capture_output=True,timeout=15)
        (root/'parser.log').write_text(parsed.stdout+'\n'+parsed.stderr,encoding='utf-8')
        # This exact SDK's `gz sdf -k` returns 0 even for malformed XML.
        # Keep that return code as evidence; acceptance depends on the actual
        # native success/error diagnostics AND the independent XML structure.
        ansi=re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
        stdout=ansi.sub('',parsed.stdout); stderr=ansi.sub('',parsed.stderr)
        native_success=any(line.strip()=='Check complete' for line in stdout.splitlines())
        native_parse_error=bool(re.search(r'\bError(?:\s*:|\s*\[)|Error parsing XML|SDF parsing the xml failed',stdout+'\n'+stderr))
        try:
            xml=ET.fromstring(owned.world.read_bytes()); result['xml_valid']=True
        except ET.ParseError as error:
            result['xml_valid']=False; result['xml_error']=str(error)
        result['parser']={'command':command,'returncode':parsed.returncode,'stdout':parsed.stdout,
                          'stderr':parsed.stderr,'native_success':native_success,
                          'native_parse_error':native_parse_error,'xml_valid':result['xml_valid'],
                          'success_marker':'Check complete',
                          'valid':parsed.returncode==0 and native_success and not native_parse_error and result['xml_valid'],
                          'tool_identity':record['parser_identity']}
        write_json(root/'parser.json',result['parser'])
        if not result['xml_valid']:
            require(native_parse_error and not native_success,'Malformed XML lacks actual native parser-error evidence; inspect parser.log')
            result.update(completed=True,physical_trial='Not started: actual native SDF parser rejected malformed bytes')
            return result
        require(result['parser']['valid'],'Native SDF validation rejected structurally parsed XML; inspect parser.log')
        w=xml.find('world')
        require(w is not None and w.get('name')==WORLD,'Use the lesson35 world name')
        require({m.get('name') for m in w.findall('model')}=={'floor','cube_000'},'Use the two local clinic models')
        require(not list(xml.iter('uri')) and not list(xml.iter('include')),'Clinic trials are self-contained and offline')
        with owned:
            entered=True; lab=Lab(owned.env,root/'native-events.jsonl'); deadline=time.monotonic()+20; errors=[]
            while True:
                require(owned.server.poll() is None,'Actual gzserver exited: inspect gzserver.out')
                try:
                    initial=lab.checkpoint(); physics=lab.physics(); entity=lab.entity(); break
                except (RuntimeError,subprocess.TimeoutExpired) as error:
                    errors.append(str(error)); require(time.monotonic()<deadline,'Native readiness failed: '+str(error))
            require(initial['sim_time_ns']==initial['iterations']==0,'Fresh isolated server did not start paused at zero')
            require(abs(float(physics['max_step_size'])-.001)<1e-12,'Use the clinic 1ms step size')
            result.update(initial=initial,native_physics=physics,native_entity=entity,readiness_errors=errors,
                          plugin_maps=plugin_maps(owned))
            if checkpoint_hook: checkpoint_hook('initial',lab,owned)
            first=lab.step(200); result['freefall']=first['after']
            if checkpoint_hook: checkpoint_hook('freefall',lab,owned)
            second=lab.step(800); result['final']=second['after']
            if checkpoint_hook: checkpoint_hook('final',lab,owned)
            result.update(completed=True,requests=[first['request'],second['request']],
                          server_log=(root/'gzserver.out').read_text(encoding='utf-8',errors='replace'))
        return result
    except BaseException as error:
        result['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        if not entered: owned.__exit__(None,None,None)
        result['ownership']=json.loads((root/'ownership.json').read_text())
        result['input_preserved']=sha(source)==original
        write_json(root/'trial.json',result)
        require(result['input_preserved'],'Input changed during observation')


def assess(observation):
    """Conjunctive causal evidence: one visible symptom alone is insufficient."""
    evidence=[]; code='UNKNOWN'
    if (not observation['parser']['valid'] and observation['parser']['native_parse_error'] and
            not observation.get('xml_valid',True)):
        code='MALFORMED_SDF'
        evidence=['Actual gz sdf -k rejected the preserved input',observation['xml_error']]
    elif observation.get('completed') and 'freefall' in observation:
        xml=ET.parse(str(Path(observation['directory'])/'world.sdf')); w=xml.find('world')
        cube=w.find("model[@name='cube_000']"); body=cube.find("link[@name='body']")
        p=observation['freefall']['positions_m']['cube_000']; final=observation['final']['positions_m']['cube_000']
        initial=observation['initial']['positions_m']['cube_000']; gravity=observation['native_physics'].get('gravity',{})
        loaded=observation['plugin_maps']['loaded_expected']; log=observation.get('server_log','')
        plugin=cube.find('plugin'); filename=plugin.get('filename','') if plugin is not None else ''
        elapsed=observation['freefall']['sim_time_ns']==200000000 and observation['freefall']['iterations']==200
        native_collisions=sum(len(link.get('collision',[])) for link in observation['native_entity'].get('link',[]))
        if (plugin is not None and filename!='libfault_force.so' and not loaded and
                ('Failed to load plugin' in log or 'cannot open shared object file' in log) and
                elapsed and abs(p[0]-initial[0])<1e-7 and p[2]<initial[2]-.1):
            code='PLUGIN_LOAD_FAILURE'; evidence=['SDF plugin filename: '+filename,'Actual gzserver reports loader failure',
                'Expected private plugin ELF is absent from actual server /proc/maps','200 real ticks: cube falls but X force effect is absent']
        elif (abs(float(gravity.get('z',0)))<1e-12 and elapsed and abs(p[2]-initial[2])<1e-7 and
                abs(final[2]-initial[2])<1e-7 and loaded and p[0]>initial[0]+.01):
            code='GRAVITY_DISABLED'; evidence=['Actual physics_info reports zero vertical gravity',
                '200 then 1000 real ticks advance, X force acts, height remains unchanged']
        elif (body.find('collision') is None and native_collisions==0 and elapsed and final[2]<-.5 and loaded):
            code='COLLISION_MISSING'; evidence=['Preserved SDF body has visual/inertia but no collision',
                'Actual entity_info body has zero collision shapes','1000 actual ticks: cube center falls below floor']
        elif (loaded and abs(float(gravity.get('z',0))+9.8)<1e-8 and native_collisions>0 and elapsed and
                abs(p[0]-initial[0]-.0201)<3e-6 and abs(p[2]-initial[2]+.19698)<3e-6 and abs(final[2]-.1)<.015):
            code='HEALTHY'; evidence=['Expected ELF actually mapped; native gravity and collision are present',
                '200-step Newton/ODE freefall response matches and 1000-step cube lands']
        else: evidence=['Observed evidence does not establish one of the four isolated clinic faults; no automatic repair']
    return {'code':code,'evidence':evidence}


def diagnose(input_path, output_dir=None, checkpoint_hook=None):
    root=Path(output_dir or (Path.cwd()/'build'/('diagnosis-'+uuid.uuid4().hex)))
    observation=trial(input_path,root,checkpoint_hook)
    result={'input':str(Path(input_path).resolve()),'input_sha256':observation['input_sha256'],
            'diagnosis':assess(observation),'observation':observation,'report_file':str(root/'diagnosis.json')}
    write_json(root/'diagnosis.json',result)
    return result


def repair(input_path, output_path, diagnosis):
    """Create only a new file, tied to the exact bytes actually diagnosed."""
    source=Path(input_path).resolve(); target=Path(output_path).resolve()
    require(not target.exists() and target!=source,'Repair requires a new output filename')
    require(sha(source)==diagnosis['input_sha256'],'Input differs from the actual diagnosed bytes')
    code=diagnosis['diagnosis']['code']; data=source.read_bytes(); method='minimal XML element repair'
    require(code in ('MALFORMED_SDF','PLUGIN_LOAD_FAILURE','GRAVITY_DISABLED','COLLISION_MISSING'),'No established repairable clinic cause')
    if code=='MALFORMED_SDF':
        if b'</world_broken>' in data:
            data=data.replace(b'</world_broken>',b'</world>',1); method='replace the mismatched world closing tag only'
            ET.fromstring(data)
        else:
            data=(Path.cwd()/'world.sdf').read_bytes(); method='explicit healthy reference copy into new file; original malformed learner file retained'
    else:
        xml=ET.fromstring(data,parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
        w=xml.find('world'); cube=w.find("model[@name='cube_000']")
        if code=='PLUGIN_LOAD_FAILURE': cube.find('plugin').set('filename','libfault_force.so')
        elif code=='GRAVITY_DISABLED': w.find('gravity').text='0 0 -9.8'
        else:
            reference=ET.parse('world.sdf').find("world/model[@name='cube_000']/link/collision")
            require(reference is not None,'Healthy reference collision unavailable')
            cube.find("link[@name='body']").append(copy.deepcopy(reference))
        data=ET.tostring(xml,encoding='utf-8',xml_declaration=True)
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('xb') as stream: stream.write(data)
    require(sha(source)==diagnosis['input_sha256'],'Repair changed original input')
    return {'input':str(source),'input_sha256':sha(source),'output':str(target),'output_sha256':sha(target),
            'method':method,'source_preserved':True,'physical_validation':'Required: rerun diagnosis on the new file'}


def run(case='all'):
    cases=CASES if case=='all' else (case,); root=Path.cwd()/'build'/('clinic-'+uuid.uuid4().hex)
    root.mkdir(parents=True); original=sha('world.sdf'); record=prepared()
    result={'passed':False,'directory':str(root),'report_file':str(root/'report.json'),'cases':[],
            'healthy_input_sha256':original,'provenance':record}
    codes=dict(zip(CASES,('MALFORMED_SDF','PLUGIN_LOAD_FAILURE','GRAVITY_DISABLED','COLLISION_MISSING')))
    try:
        for name in cases:
            folder=root/name; folder.mkdir(); bad=folder/'bad.sdf'; fixed=folder/'repaired.sdf'
            bad.write_bytes(inject(Path('world.sdf').read_bytes(),name)); before=diagnose(bad,folder/'before')
            require(before['diagnosis']['code']==codes[name],'Actual evidence did not establish expected cause: '+name)
            change=repair(bad,fixed,before); after=diagnose(fixed,folder/'after')
            require(after['diagnosis']['code']=='HEALTHY','Repaired input did not restore actual physics: '+name)
            result['cases'].append({'case':name,'before':before,'repair':change,'after':after,'passed':True})
            write_json(root/'report.json',result)
        require(sha('world.sdf')==original,'Clinic changed GUI preview or learner input')
        result['passed']=True; return result
    finally: write_json(root/'report.json',result)


def main():
    parser=argparse.ArgumentParser(description=__doc__); actions=parser.add_subparsers(dest='action',required=True)
    one=actions.add_parser('run'); one.add_argument('--case',choices=('all',)+CASES,default='all')
    one=actions.add_parser('inject'); one.add_argument('--case',choices=CASES,required=True); one.add_argument('--out',required=True)
    one=actions.add_parser('diagnose'); one.add_argument('input'); one.add_argument('--out')
    one=actions.add_parser('repair'); one.add_argument('input'); one.add_argument('--out',required=True); one.add_argument('--report',required=True)
    args=parser.parse_args()
    if args.action=='run': result=run(args.case)
    elif args.action=='diagnose': result=diagnose(args.input,args.out)
    elif args.action=='repair': result=repair(args.input,args.out,json.loads(Path(args.report).read_text()))
    else:
        path=Path(args.out); path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('xb') as stream: stream.write(inject(Path('world.sdf').read_bytes(),args.case))
        result={'injected':args.case,'output':str(path.resolve()),'sha256':sha(path)}
    # All full evidence stays on disk; the GUI console receives a useful locator.
    print(json.dumps({'passed':result.get('passed',True),'diagnosis':result.get('diagnosis'),
                      'report_file':result.get('report_file'),'result':result if args.action in ('repair','inject') else None},ensure_ascii=False))


if __name__=='__main__':
    try: main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False)); raise SystemExit(1)
