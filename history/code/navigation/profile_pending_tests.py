#!/usr/bin/env python3
"""Read-only exact-profile/old-reference/pending-launch checks; never launch ROS."""
import argparse,hashlib,json
from pathlib import Path
from full46_launch_contract import PROFILE_NAMES,expected_profile,validate_full46,normal_form,baseline_profile_path
from corridor_preflight import verify_preflight,verify_original_v25,verify_original_v26
HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();checks=[]
 for n in PROFILE_NAMES:
  p=json.loads((HERE/'profiles'/n).read_text());validate_full46(p,HERE)
  checks.append(dict(name=n+'_exact_profile_and_V25_normalform',passed=p==expected_profile(HERE,n)and normal_form(p,HERE)==json.loads(baseline_profile_path(HERE).read_text())))
 try:verify_preflight(HERE,p)
 except RuntimeError as e:checks.append(dict(name='pending_V28_gate_cannot_launch',passed='Incomplete independent V28' in str(e),actual_error=str(e)))
 else:checks.append(dict(name='pending_V28_gate_cannot_launch',passed=False))
 verify_original_v26(HERE);checks.append(dict(name='original_V26_reference_bytes_unchanged',passed=True))
 v25=verify_original_v25(HERE);checks.append(dict(name='original_V25_gate_still_valid',passed=True,witness=v25))
 for name in ['path_admission.py','cascade_core.py','teacher_wrapper.py','worker.py','bridge.py','transition_gate.py','replan_policy.py','clock_hold.py','mission46.py']:
  x=sha(HERE/name);checks.append(dict(name=name+'_byte_identical_frozen_steady',passed=x==sha(HERE.parent/'path_admission_v27_steady'/name),sha256=x))
 for name in ['controller.py','shared_controller.py']:
  compile((HERE/name).read_text(),str(HERE/name),'exec');checks.append(dict(name=name+'_valid_python',passed=True))
 result=dict(schema='V28_capture_profiles_and_pending_contract/v1',passed=all(x['passed']for x in checks),checks=checks,simulation_launched=False,script_sha256=sha(__file__))
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(passed=result['passed'],checks=len(checks))))
 if not result['passed']:raise SystemExit(1)
if __name__=='__main__':main()
