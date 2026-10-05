"""Read actual r2 command evidence and plot the conflicting heading references."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
RUN=HERE.parents[2]/'runs/20261006_022511_closed_loop_cascade_clock_hold_v19_original46_r2_4fc5'
SOURCE=RUN/'navigation_pid_history.jsonl'
rows=[]
digest=hashlib.sha256()
with SOURCE.open('rb') as stream:
    for line in stream:
        digest.update(line)
        d=json.loads(line); c=d.get('cascade',{}); gate=d.get('heading_gate_reference',{})
        if d.get('waypoint_index')!=8 or not c.get('controller_updated') or 'reference_yaw_rad' not in c:
            continue
        locked=gate.get('locked_heading')
        if locked is None: continue
        rows.append((d['control_stamp_ns']/1e9,locked,c['reference_yaw_rad'],
                     c['reference_yaw_rad']-c['error_yaw_rad'],c['command_body'][0],c['command_body'][2]))
fig,ax=plt.subplots(2,1,figsize=(10,6),sharex=True,constrained_layout=True)
x=[r[0]for r in rows]
for index,label,color in [(1,'Outer gate locked heading','#a54537'),(2,'Cascade SCAN heading','#126f88'),(3,'Actual SLAM yaw','#8c8743')]:
    ax[0].plot(x,[r[index]for r in rows],label=label,color=color,lw=1.5)
ax[0].set_ylabel('Heading (rad)');ax[0].legend(fontsize=9);ax[0].grid(alpha=.2)
ax[0].set_title('Original46 r2: after replanning, the two heading owners disagree')
ax[1].plot(x,[r[4]for r in rows],label='Forward command (m/s)',color='#126f88')
ax[1].plot(x,[r[5]for r in rows],label='Yaw command (rad/s)',color='#a54537')
ax[1].set_xlabel('Actual simulation clock (s)');ax[1].set_ylabel('Command');ax[1].grid(alpha=.2);ax[1].legend()
fig.savefig(HERE/'heading_reference_failure.png',dpi=160)
(HERE/'heading_plot_source.json').write_text(json.dumps(dict(run_id=RUN.name,source=str(SOURCE),
    source_sha256=digest.hexdigest(),samples=len(rows),waypoint_index=8,
    source_kind='Actual recorded SLAM feedback and command producer; no simulated trajectory',
    original_run_modified=False),indent=2)+'\n')
