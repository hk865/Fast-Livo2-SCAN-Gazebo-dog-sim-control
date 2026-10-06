"""Plot descriptive replay evidence; no fabricated trajectory or speedup."""
from pathlib import Path
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
source = ROOT.parent / 'performance/CAPACITY_1P5X_COMPARISON.json'
data = json.loads(source.read_text())
fig, axes = plt.subplots(1, 2, figsize=(11, 4.3), constrained_layout=True)
colors = {'serial': '#a54537', 'staged': '#126f88'}
for row in data['rows']:
    bins = row['age_by_5s_source_bin']
    x = [(r['source_begin_s'] + r['source_end_s'])/2 for r in bins]
    y = [r['p95_ms'] for r in bins]
    for ax in axes:
        ax.plot(x, y, marker='o', ms=3, label=row['mode'], color=colors[row['mode']])
        ax.set_xlabel('Original acquisition time (s)')
        ax.grid(alpha=.2)
axes[0].axhline(300, color='#555', ls='--', lw=1, label='Controller TTL reference (300 ms)')
axes[0].set_ylabel('Observer clock - pose stamp (ms)')
axes[0].set_title('Same bag, 1.5x release: p95 in each 5 s bin')
axes[0].legend(fontsize=8)
axes[1].set_ylim(0, 65)
axes[1].set_title('Low-latency detail (same measurements)')
axes[1].set_ylabel('Observer delay (ms)')
fig.suptitle('V19 receive/decode pipeline: limited capacity probe', fontsize=14)
fig.text(.5, -.055, 'One process per mode; no Gazebo/Teacher; topic interleaving differs. Not a general speedup or navigation PASS.', ha='center', fontsize=9)
fig.savefig(ROOT/'capacity_probe.png', dpi=160, bbox_inches='tight')
(ROOT/'capacity_plot_source.json').write_text(json.dumps({
    'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'statistic': 'p95 of observer clock minus original pose header in each source-time 5s bin',
    'independent_runs_per_mode': 1, 'general_speedup_verified': False,
}, indent=2)+'\n')
