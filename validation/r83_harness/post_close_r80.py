# Publication 2026-10-13: local paths redacted; numeric/control behavior retained; local admission is not transferred.
"""Read-only host closure followed by original budget accounting, only after owner exits."""
from pathlib import Path
import datetime, hashlib, importlib.util, json, sys
S=Path('./validation/r83_harness')
def read(p):return json.loads(p.read_text())
def write(p,d):
    with p.open('x') as f:json.dump(d,f,indent=2);f.write('\n')
R=Path(read(S/'PREPARED_RUN.json')['run'])
outcome=read(S/'RUN_OUTCOME.json')
m=read(R/'runtime_manifest.json')
groups={p['pgid'] for p in m['owned_processes']}
groups.add(m['independent_owned_guardian']['identity']['pgid'])
owner_pids={p['ppid'] for p in m['owned_processes']}
processes=[]
for d in Path('/proc').iterdir():
    if not d.name.isdigit():continue
    try:
        st=d.joinpath('stat').read_text();tail=st[st.rfind(')')+2:].split()
        args=d.joinpath('cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
        processes.append(dict(pid=int(d.name),pgid=int(tail[2]),state=tail[0],start_ticks=int(tail[19]),argv=args))
    except (OSError,ValueError,IndexError):continue
remaining=[p for p in processes if p['pgid'] in groups]
simroles=[p for p in processes if (any(Path(a).name=='gz' for a in p['argv']) and 'sim' in p['argv']) or any(Path(a).name in ('fastlivo_mapping','ipc_request_window.py','shadow.launch.py','stack.launch.py') for a in p['argv'])]
owner_present=any(p['pid'] in owner_pids for p in processes)
assert not owner_present,'Original owner still present; no post-close accounting yet'
assert not remaining,'Owned groups still present; report instead of signaling'
host=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_host_proc=True,process_count=len(processes),owned_and_guardian_groups=sorted(groups),remaining_group_members=remaining,other_simulation_roles=simroles,all_owned_and_guardian_groups_absent=not remaining,all_simulation_roles_absent=not simroles,original_owner_pid_present=owner_present,no_signals_sent=True)
write(S/'HOST_PROCESS_CLOSURE.json',host)
sys.path.insert(0,str(S))
spec=importlib.util.spec_from_file_location('storage_guard',S/'storage_guard.py');guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
rows=[]
p=R/'storage_observations.jsonl'
if p.exists():rows=[json.loads(line)for line in p.read_text().splitlines()if line.strip()]
p=R/'guardian_storage_last.json'
if p.exists():rows.append(read(p))
previous=max(rows,key=lambda d:d.get('R77_round_budget',{}).get('charged_B',0))if rows else None
budget=guard.observe(R,m['profile'],previous=previous)
write(S/'POSTROUND_BUDGET.json',budget)
print(json.dumps(dict(host_closed=not remaining,other_sim_roles=len(simroles),shared_B=budget['conservative_session_growth_bytes'],remaining_B=130000000000-budget['conservative_session_growth_bytes'],round_B=budget['R77_round_budget']['charged_B'],stop_reasons=budget['stop_reasons'])))
