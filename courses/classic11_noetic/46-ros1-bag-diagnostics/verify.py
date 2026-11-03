#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys
source=Path(os.environ.get('GAZEBOTEACH_LESSON_SOURCE',Path(__file__).resolve().parent)).resolve();sys.path.insert(0,str(source))
from instrument import verify
if __name__=='__main__':
    result=verify()
    # Full high-rate Clock rows stay in the audit file; keep the live terminal
    # readable while retaining the real verifier's exit/failure behavior.
    checks=result['checks']
    print(json.dumps({'passed':result['passed'],'assignment_pass':result['assignment_pass'],
        'bag_path':result['bag_path'],'bag_sha256':result['bag_sha256'],
        'recorded_counts':checks['real_recording']['audit']['counts'],
        'replay_wall_span_ratio':checks['real_cli_replays']['measured_1x_to_2x_wall_span_ratio'],
        'diagnostic_levels':[row['level'] for row in checks['actual_stale_stream_diagnosis']['dropout_diagnostics']],
        'full_report':str(Path('build/verification.json').resolve())},
        ensure_ascii=False,indent=2,allow_nan=False))
