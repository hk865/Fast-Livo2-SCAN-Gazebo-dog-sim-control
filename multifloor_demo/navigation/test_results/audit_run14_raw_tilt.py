#!/usr/bin/env python3
"""Read-only exact raw IMU tilt crossing audit, using the production definition."""
import hashlib,json,sys
from pathlib import Path
from scipy.spatial.transform import Rotation
NAV=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(NAV))
from control_core import body_tilt
run=NAV.parent/'runs/20261001_174341_6cd895'
crossings=[];blocks=[];active=None;last=None
for line in (run/'sensor_audit.jsonl').open():
    row=json.loads(line)
    if row.get('source')!='imu' or not 350<=row['stamp']<=365:continue
    tilt=body_tilt(Rotation.from_quat(row['quaternion']).as_matrix())
    if tilt>=.30:
        if active is None:active=dict(first_stamp=row['stamp'],max_tilt=tilt,max_stamp=row['stamp'],last_stamp=row['stamp'],samples=0)
        active['last_stamp']=row['stamp'];active['samples']+=1
        if tilt>active['max_tilt']:active.update(max_tilt=tilt,max_stamp=row['stamp'])
    elif active is not None:blocks.append(active);active=None
    if tilt>=.30 and (last is None or last<.30):crossings.append(dict(stamp=row['stamp'],tilt=tilt,angular_velocity=row['angular_velocity']))
    last=tilt
if active:blocks.append(active)
report=dict(scope=__doc__,definition='max(abs(roll),abs(pitch)), identical to production body_tilt; fixed body/IMU extrinsic identity',
    input_SHA256=hashlib.sha256((run/'sensor_audit.jsonl').read_bytes()).hexdigest(),threshold=.30,crossings=crossings,above_threshold_blocks=blocks)
out=NAV/'test_results/run14_raw_tilt_events.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
