import json
import os
import subprocess
import time

def sample():
    text=subprocess.check_output(['gz','topic','-e','-t','/world/default/pose/info','-n','1','--json-output'],text=True,timeout=6)
    payload=json.JSONDecoder().raw_decode(text[text.index('{'):])[0]
    return next(row for row in payload['pose'] if row.get('name')=='probe')

time.sleep(1)
value=sample();z=float(value.get('position',{}).get('z',0))
assert abs(z-.1)<.04, {'expected_z':.1,'actual_z':z}
print(json.dumps({'passed':True,'probe_z':z,'partition':os.environ.get('GZ_PARTITION')}),flush=True)
