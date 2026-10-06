#!/usr/bin/env python3
"""Summarize stored actual internal states and timings; no ROS or control."""
from collections import defaultdict
import json
from pathlib import Path
import re
import sys
import numpy as np


def analyze(directory):
    root=Path(directory)
    report=json.loads((root/'probe_report.json').read_text())
    expression=r'\[DEMO_SYNC\] camera=([\d.]+) lidar_newest=([\d.]+) imu_newest=([\d.]+) imu_last_used=([-\d.]+) imu_count=(\d+) complete=(\d+)'
    sync=np.array([[float(value) for value in match] for match in re.findall(expression,(root/'stack.log').read_text())])
    report['internal_sync']={'frames':len(sync),'max_camera_minus_newest_imu_s':float((sync[:,0]-sync[:,2]).max()),
        'max_camera_minus_last_used_imu_s':float((sync[sync[:,4]>0,0]-sync[sync[:,4]>0,3]).max()),
        'incomplete_more_than_1ms':int((sync[:,0]-sync[:,2]>.001).sum()),
        'min_nonempty_imu_samples':int(sync[sync[:,4]>0,4].min()),
        'empty_imu_frames':int((sync[:,4]==0).sum()),
        'empty_imu_camera_stamps':sync[sync[:,4]==0,0].tolist()}
    pre=np.loadtxt(root/'fastlivo_mat_pre.txt');out=np.loadtxt(root/'fastlivo_mat_out.txt')
    groups=[defaultdict(list),defaultdict(list)]
    for data,group in zip((pre,out),groups):
        for row in data:group[round(row[0],6)].append(row)
    updates=defaultdict(list)
    for ts,rows in groups[1].items():
        if len(rows)!=2 or len(groups[0].get(ts,[]))!=2:continue
        for index,name in enumerate(('LIO','VIO')):
            updates[name].append(float(np.linalg.norm(rows[index][4:7]-groups[0][ts][index][4:7])))
    report['internal_position_updates_m']={kind:{'count':len(values),'max':max(values),
        'median':float(np.median(values)),'p99':float(np.quantile(values,.99))} for kind,values in updates.items()}
    records=[json.loads(line) for line in (root/'sensor_audit.jsonl').read_text().splitlines()]
    lidar=[row for row in records if row['source']=='lidar']
    report['actual_self_returns']={'scans':len(lidar),'mean_points':float(np.mean([r['points'] for r in lidar])),
        'mean_self_points':float(np.mean([r['self_points'] for r in lidar])),
        'mean_self_fraction':float(np.mean([r['self_points']/r['points'] for r in lidar if r['points']]))}
    imu=[row for row in records if row['source']=='imu']
    report['actual_physical_imu']={'acc_norm_max_m_s2':float(np.linalg.norm([r['acc'] for r in imu],axis=1).max()),
        'omega_norm_max_rad_s':float(np.linalg.norm([r['omega'] for r in imu],axis=1).max())}
    metadata=json.loads((root/'map_metadata.json').read_text())
    report['actual_rgb_map']={key:metadata.get(key) for key in ('point_count','capacity_rejections','observed_rgb_samples','error')}
    (root/'turn_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    for path in sys.argv[1:]:print(json.dumps({'directory':path,'report':analyze(path)},indent=2))
