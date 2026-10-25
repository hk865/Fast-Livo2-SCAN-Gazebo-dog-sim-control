"""Instantiate exact R73/profile paths for an isolated R102 candidate; no validator changes."""
from pathlib import Path
import argparse, hashlib, json, subprocess
BASE = "45396505e8facd808e38c28d674aa5a5c9c25947"
ORIGINAL = Path("/opt/experiment/workspace/multifloor_demo/development/r86_feedback_isolation/source/candidate")
PROFILE = ORIGINAL.parent.parent / "implementation_round23/physical/PROFILE.json"
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--profile-output",required=True); ap.add_argument("--receipt-output",required=True); args=ap.parse_args()
    root=Path(__file__).resolve().parent; candidate=root/"candidate"
    subprocess.run(["git","merge-base","--is-ancestor",BASE,"HEAD"],cwd=root,check=True)
    out=Path(args.profile_output); receipt=Path(args.receipt_output)
    for x in (out,receipt):
        x.parent.mkdir(parents=True,exist_ok=True)
        if x.exists(): raise RuntimeError("Fresh output required: "+str(x))
    profile=json.loads(PROFILE.read_bytes()); before=json.loads(json.dumps(profile))
    contract=candidate/"R73_SCAN_RUNTIME_CONTRACT.json"; value=json.loads(contract.read_bytes())
    if value["source_root"] != str(ORIGINAL): raise RuntimeError("Expected original binding, not an already rebound contract")
    policy=candidate/"POLICY_SELECTION_CONTRACT.json"; policy_value=json.loads(policy.read_bytes())
    if policy_value["candidate_origin"] != str(ORIGINAL/"policy_candidate/worker.py"): raise RuntimeError("Unexpected private policy path")
    original_policy_sha=sha(policy); policy_value["candidate_origin"]=str(candidate/"policy_candidate/worker.py")
    policy_raw=(json.dumps(policy_value,indent=2)+"\n").encode()
    original_contract_sha=sha(contract); value["source_root"]=str(candidate)
    # Preserve every dependency binding/basis/ELF/workspace field byte value.
    contract_raw=(json.dumps(value,indent=2)+"\n").encode()
    s=profile["r85_session"]
    for key in ("resource_renewal_proof", "scan_workspace_contract"):
        old=Path(s[key]); new=candidate/old.name
        if old.parent != ORIGINAL: raise RuntimeError("Unexpected path role: "+key)
        s[key]=str(new); s[key+"_sha256"]=(hashlib.sha256(contract_raw).hexdigest() if new==contract else sha(new))
    a=json.loads(json.dumps(before)); b=json.loads(json.dumps(profile))
    for x in (a,b):
        for key in ("resource_renewal_proof", "scan_workspace_contract"):
            x["r85_session"].pop(key); x["r85_session"].pop(key+"_sha256")
    if a!=b: raise RuntimeError("Business profile changed")
    # Validate all metadata and destination preconditions before any contract mutation.
    pending=contract.with_suffix('.json.r102-pending')
    with pending.open('xb') as f: f.write(contract_raw)
    pending.replace(contract)
    pending_policy=policy.with_suffix('.json.r102-pending')
    with pending_policy.open('xb') as f: f.write(policy_raw)
    pending_policy.replace(policy)
    with out.open("x") as f: json.dump(profile,f,indent=2); f.write("\n")
    with receipt.open("x") as f:
        json.dump(dict(schema="r102_exact_worktree_path_binding/v1",baseline=BASE,worktree=str(root),candidate=str(candidate),original_profile=str(PROFILE),original_profile_sha256=sha(PROFILE),profile=str(out),profile_sha256=sha(out),original_contract_sha256=original_contract_sha,new_contract_sha256=sha(contract),contract_delta_only_source_root=True,policy_contract_delta_only_candidate_origin=True,original_policy_contract_sha256=original_policy_sha,new_policy_contract_sha256=sha(policy),profile_delta_only_exact_path_and_contract_hash=True,validators_unchanged=True),f,indent=2); f.write("\n")
if __name__=="__main__": main()
