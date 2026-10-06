"""Only the owned-clean actual candidate; missing goal8 is not invented."""
import importlib.util
import json
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('phases',HERE/'audit_phase.py')
phases=importlib.util.module_from_spec(spec)
spec.loader.exec_module(phases)
run=Path(sys.argv[1]).resolve()
output=Path(sys.argv[2]).resolve()
assert not output.exists()
result=dict(scope=__doc__,candidate=phases.analyze(run),
    candidate_failed_goal7=phases.analyze(run,6),candidate_goal8=None,
    goal8_missing_reason='No actual index7 entry and no preceding goal6 receipt; safety failed at index6.')
output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:dict(interval_ns=v['observed_interval_ns'],durations=v['phase_support_duration_s'],
    groups={name:{'seconds':r['duration_sim_s'],'vx_integral':r['emitted_vx_integral_m'],
        'yaw_integral':r['emitted_wz_integral_rad'],'SLAM_body_forward':r['SLAM']['body_forward_m'],
        'GT_body_forward':r['GT_evaluation']['body_forward_m']} for name,r in v['groups'].items()})
    for k,v in result.items() if isinstance(v,dict)},indent=2))
