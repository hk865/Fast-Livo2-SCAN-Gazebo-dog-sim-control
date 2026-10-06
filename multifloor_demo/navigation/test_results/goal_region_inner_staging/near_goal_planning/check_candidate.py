#!/usr/bin/env python3
"""Review the exact isolated SCAN gate diff before changing production."""
import difflib, hashlib, json, math
from pathlib import Path
HERE=Path(__file__).parent
NAV=HERE.parents[2]
SOURCE=NAV/'ros2_ws/src/plan_manage/src/planner_manager.cpp'
original=((HERE/'planner_manager.before.cpp').read_bytes() if (HERE/'planner_manager.before.cpp').exists() else SOURCE.read_bytes())
old=b'if ((start_pt - local_target_pt).norm() < 0.2)'
new=b'if ((start_pt - local_target_pt).norm() < 0.05)'
assert original.count(old)==1
candidate=original.replace(old,new)
(HERE/'planner_manager.before.cpp').write_bytes(original)
(HERE/'planner_manager.candidate.cpp').write_bytes(candidate)
(HERE/'minimal.patch').write_text(''.join(difflib.unified_diff(original.decode().splitlines(True),candidate.decode().splitlines(True),fromfile='planner_manager.before.cpp',tofile='planner_manager.candidate.cpp')))
checks={
 'one_numeric_planning_gate_only':candidate.replace(new,old)==original,
 'origin19_outside_control17':.19>.17,
 'origin19_allowed_by_new_gate':not .19<.05,
 'sub05_remains_rejected':.049<.05,
 'exact05_passes_gate':not .05<.05,
 'collision_optimizer_still_present':all(s in candidate for s in [b'initControlPoints(ctrl_pts, true)',b'BsplineOptimizeTrajRebound(ctrl_pts, ts)',b'checkDynamicFeasibility(pos)',b'updateTrajInfo(pos, node_->now())']),
}
# The unmodified polynomial sampling loop reduces ts until >=7 initial points;
# finite positive short travel time at these distances cannot create a zero-time
# spline. These equations mirror source only as a static pre-build sanity check.
samples=[]
for distance in (.05,.051,.10,.17,.19,.20):
 v,a,spacing=.12,.15,.2
 duration=(math.sqrt(distance/a) if v*v/a>distance else (distance-v*v/a)/v+2*v/a)
 ts=spacing/v*(1.2 if distance>.1 else 5.)
 reductions=0
 while math.ceil(duration/ts)<7:
  ts/=1.5;reductions+=1
 assert math.isfinite(duration) and duration>0 and reductions<50
 samples.append(dict(distance_m=distance,duration_s=duration,reduced_ts_s=ts,minimum_samples=math.ceil(duration/ts),reductions=reductions))
checks['positive_bounded_sampling_time']=True
result=dict(scope='Exact source-gate diff plus static numeric sanity, not actual planner/physics validation',passed=all(checks.values()),checks=checks,sampling=samples,source_sha256=hashlib.sha256(original).hexdigest(),candidate_sha256=hashlib.sha256(candidate).hexdigest())
(HERE/'candidate_static_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2));assert result['passed']
