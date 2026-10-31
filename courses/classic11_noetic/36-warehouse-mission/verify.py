#!/usr/bin/env python3
"""A finite physical mission; independent acceptance also checks fault recovery."""
import csv
import json
import math
from pathlib import Path
from std_srvs.srv import Trigger,Empty
from lab import Client


def main():
    client=Client();client.ready();client.pause(True)
    client.call('/gazebo/reset_simulation',Empty)
    client.call('/lesson36/reset',Trigger)
    client.pause(False)
    try:
        initial=client.state['sim_time'];client.wait(lambda:client.state['x'] is not None and client.state['sim_time']>initial+.6,15)
        run_id=client.call('/lesson36/start',Trigger).message
        state=client.wait(lambda:client.state['run_id']==run_id and client.state['phase'] in ('SUCCEEDED','FAILED'),100)
    finally: client.pause(True)
    directory=Path(state['directory']);deadline=__import__('time').monotonic()+5
    while not (directory/'result.json').exists() and __import__('time').monotonic()<deadline: __import__('time').sleep(.02)
    result=json.loads((directory/'result.json').read_text())
    if result['phase']!='SUCCEEDED': raise RuntimeError(result['reason'])
    stages=[item['data']['to'] for item in result['transitions']]
    if stages!=['SCAN','APPROACH','WAIT_DOOR','PASS','GOAL','STOPPING','SUCCEEDED']: raise RuntimeError('Incomplete finite mission flow')
    if result['bumper_contacts'] or result['point_count']<500: raise RuntimeError('Collision or insufficient measured scan points')
    rows=list(csv.DictReader((directory/'points.csv').open()))
    if len(rows)!=result['point_count']: raise RuntimeError('Frozen measured points/result count differs')
    for row in rows:
        a=float(row['robot_yaw_rad']);r=float(row['range_m']);t=float(row['angle_rad'])
        x=float(row['robot_x_m'])+math.cos(a)*(.15+r*math.cos(t))-math.sin(a)*r*math.sin(t)
        if abs(x-float(row['x_m']))>1e-10: raise RuntimeError('Projection arithmetic differs')
    print(json.dumps({'verified':True,'actual_result':result,'note':'Actual physics acceptance; not SLAM/nav-stack validation'},ensure_ascii=False))


if __name__=='__main__': main()
