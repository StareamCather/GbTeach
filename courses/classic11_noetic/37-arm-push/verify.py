#!/usr/bin/env python3
"""Finite real-physics run and measured FK/force/goal consistency checks."""
import csv
import json
import math
from pathlib import Path
from std_srvs.srv import Trigger,Empty
from lab import Client


def main():
    client=Client();client.ready();client.pause(True)
    client.call('/gazebo/reset_world',Empty);client.call('/lesson37/reset',Trigger)
    client.pause(False)
    try:
        initial=client.state['sim_time']
        client.wait(lambda:client.state['force'] is not None and client.state['sim_time']>initial+.6,15)
        run_id=client.call('/lesson37/start',Trigger).message
        value=client.await_result(run_id)
    finally:client.pause(True)
    directory=Path(value['state']['directory']);result=json.loads((directory/'result.json').read_text())
    if result['phase']!='SUCCEEDED':raise RuntimeError(result['reason'])
    stages=[item['data']['to'] for item in result['transitions']]
    if stages!=['SETTLE','PUSH','HOLD','STOPPING','SUCCEEDED']:raise RuntimeError('Incomplete physical mission flow')
    rows=list(csv.DictReader((directory/'measurements.csv').open()))
    if len(rows)!=result['measurement_count'] or len(rows)<15:raise RuntimeError('Insufficient measured rows')
    alpha=math.acos(4/7)
    for row in rows:
        a=float(row['q1_rad'])-alpha;b=float(row['q2_rad'])+2*alpha
        x=.35*math.cos(a)+.35*math.cos(a+b);y=.35*math.sin(a)+.35*math.sin(a+b)
        if math.hypot(x-float(row['tip_x_m']),y-float(row['tip_y_m']))>.012:
            raise RuntimeError('Actual joint FK and independently queried pusher differ')
        error=math.hypot(float(row['tip_x_m'])-float(row['desired_x_m']),float(row['tip_y_m'])-float(row['desired_y_m']))
        if abs(error-float(row['error_m']))>1e-9:raise RuntimeError('Actual tool tracking error arithmetic differs')
    if result['truth']['block_x']-result['initial_block_x']<=.1:raise RuntimeError('Actual block did not move .1m')
    if result['truth']['error']>=.025 or not result['contact_count'] or result['peak_horizontal_force_n']<=.12:
        raise RuntimeError('Actual tip/contact/force goal failed')
    print(json.dumps({'verified':True,'actual_result':result,'note':'Real 2R physical push; force is stock joint FT, not a calculated proxy'},ensure_ascii=False))


if __name__=='__main__':main()
