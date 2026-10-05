"""Actual SLAM traces, in each run's own camera_init coordinates."""
from pathlib import Path
import json
import hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
RUNS=HERE.parents[2]/'runs'
IDS=['20261006_022511_closed_loop_cascade_clock_hold_v19_original46_r2_4fc5',
     '20261006_024447_closed_loop_cascade_clock_hold_v19_original46_r3_heading_9779']
fig,axes=plt.subplots(1,2,figsize=(11,5),constrained_layout=True)
bindings={}
for ax,run_id,label in zip(axes,IDS,['r2: mismatched turn targets','r3: turn targets corrected']):
    run=RUNS/run_id
    source=run/'navigation_slam_poses.jsonl'
    poses=[json.loads(line)['position'] for line in source.open()]
    request=json.loads((run/'navigation_request.json').read_text())
    centers=[g['center'] for g in request['goals']]
    ax.plot([p[0]for p in centers],[p[1]for p in centers],ls='--',color='#aaa',label='Original exploration centers')
    ax.plot([p[0]for p in poses],[p[1]for p in poses],color='#126f88',lw=1.5,label='Actual SLAM positions')
    ax.scatter([poses[-1][0]],[poses[-1][1]],color='#a54537',s=35,label='Final recorded pose')
    ax.scatter([p[0]for p in centers[:8]],[p[1]for p in centers[:8]],color='#7d8644',s=17,label='8 received arrivals')
    ax.set(title=label,xlabel='camera_init x (m)',ylabel='camera_init y (m)')
    ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    bindings[run_id]={str(source):hashlib.sha256(source.read_bytes()).hexdigest(),
       str(run/'navigation_request.json'):hashlib.sha256((run/'navigation_request.json').read_bytes()).hexdigest()}
fig.suptitle('Original46 attempts: both ended before the 9th exploration region',fontsize=13)
fig.savefig(HERE/'actual_slam_routes.png',dpi=160)
(HERE/'route_plot_sources.json').write_text(json.dumps(dict(source_bindings=bindings,
    ground_truth_navigation_used=False,scope='Each panel uses its own actual SLAM frame; dashed future route is not executed progress.'),indent=2)+'\n')
