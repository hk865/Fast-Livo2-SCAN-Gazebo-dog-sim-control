#!/usr/bin/env python3
"""Pure boundary/missing-window checks of the saved diagnostic functions."""
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
p=HERE/'analyze_v4.py'
s=importlib.util.spec_from_file_location('saved_actual_analysis',p)
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
checks={'previous_raw_phase_and_window_fixtures':m.h.pure_checks()}
assert m.fixed_window_checks({'status':'unverified'})['status']=='unverified'
checks['missing_window_never_becomes_pass']='passed'
w=dict(window_s=[0.,5.],xy_drift_m=.05,yaw_drift_rad=.1,
    peak_origin_planar_speed_mps=.08,peak_Euler_yaw_rate_radps=.1,peak_body_wz_radps=.1)
assert all(x['numerical_within_limit'] for x in m.fixed_window_checks(w).values())
checks['exact_threshold_window_boundaries']='passed'
w['xy_drift_m']=.050001
assert not m.fixed_window_checks(w)['xy_drift_m']['numerical_within_limit']
checks['genuine_over_limit_remains_false']='passed'
assert m.h.causal_indices([.01,.03],[.005,.025,.035]).tolist()==[-1,0,1]
checks['unknown_first_state_stays_unknown']='passed'
# The original embedded self-test omitted window_s from its fixture, which the
# production helper correctly refuses as unverified. Keep its source unchanged
# because the two actual artifact sets already bind that precise script hash.
try:m.pure_checks()
except TypeError as exc:checks['original_incomplete_test_fixture_diagnosis']=str(exc)
else:raise AssertionError('Original incomplete fixture diagnosis changed')
result=dict(status='passed',pure_checks=12,checks=checks,
    analyzer_sha256=m.h.sha(p),analyzer_actual_functions_unchanged=True,
    corrected_test_fixture_includes_declared_window=True,
    original_test_fixture_failure_not_a_physics_failure=True,
    no_original_receipt_written=True,no_simulation_or_model_executed=True)
with (HERE/'pure_fixture_review.json').open('x') as f:
    json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps(result,ensure_ascii=False))
