"""End only this owned physics run after an evidenced irrecoverable input wait.

No navigation result is fabricated. The unchanged runner retains all cleanup,
archive deadlines and physical guards; transport EOF after this operator action
is reported as such, not as a spontaneous physical fault.
"""
from pathlib import Path
import datetime as dt
import hashlib
import json
import os
import signal
import sys
import time

B=Path(__file__).resolve().parent
RUN=Path('/opt/experiment/workspace/multifloor_demo/teacher_mode/runs/20261015T154745Z_closed_loop_R85_support_suffix_r01_low_speed_20261015_cc45')
C=RUN.parents[1]/'navigation/engineering_v66_r85_turn_handoff_candidate'
sys.path.insert(0,str(C))
from owned_guardian import ident,validated_members,same

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def now():return dt.datetime.now(dt.timezone.utc).isoformat()
assert os.getuid()==os.geteuid()==1000
assert not (RUN/'EXTERNAL_OWNER_PHYSICS_STOP.json').exists()
assert not (RUN/'runtime_manifest.json').exists()
saved=read(RUN/'owned_processes_started.json')['gazebo']
permit=read(RUN/'owned_roles/gazebo.permit.json')
owner=read(RUN/'guardian_owner.json')
assert saved==permit['identity']==read(RUN/'owned_roles/gazebo.identity.json')
assert saved['pid']==saved['pgid']==saved['session']==1151343
assert saved['ppid']==1150963 and saved['owned_run']==str(RUN)
assert saved['ownership_token']==permit['token']==owner['token']
current=ident(saved['pid']);assert same(current,saved) and current['state']!='Z'
assert same(ident(owner['parent']['pid']),owner['parent'])
raw_cmdline=(Path('/proc')/str(saved['pid'])/'cmdline').read_bytes()
# Ruby/Gazebo uses setproctitle: its observed command is one NUL-terminated
# title instead of the original split argv. Require that exact owned title.
assert raw_cmdline.rstrip(b'\0')==('gz sim -s -r '+str(RUN/'world.sdf')+' --seed 42').encode()
members=validated_members(saved);assert members
log=RUN/'ros_logs/scan_planner_node_1154583_1792079734593.log'
assert 'Rejected reference: frame/finite/odometry/nonzero-segment validation failed' in log.read_text()
nav=read(RUN/'navigation_status.json')
assert nav['state']=='running' and nav.get('active_trajectory_id') is None
receipt={'schema':'r85_explicit_operator_owned_physics_stop/v1','UTC':now(),
    'run':str(RUN),'reason':'new SCAN reference rejected; upstream waits indefinitely for an uncommitted reference; original measured-stop already passes',
    'rejection_log':str(log),'rejection_log_sha256':sha(log),
    'predicate_attribution':'frame/finite/internal nonzero geometry pass offline checks; child odometry readiness remains inferred because rejection combines predicates',
    'operator_action':'SIGINT only identity-verified owned Gazebo process group',
    'identity':saved,'verified_group_members_before_signal':members,
    'signal':int(signal.SIGINT),'signal_sent':False,
    'actual_navigation_results_modified':False,'physical_algorithm_modified':False,
    'transport_EOF_after_this_action_is_not_a_spontaneous_physical_fault':True,
    'formal_acceptance_pass':False}
with (RUN/'EXTERNAL_OWNER_PHYSICS_STOP.json').open('x')as f:
    json.dump(receipt,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
os.killpg(saved['pgid'],signal.SIGINT)
receipt.update(signal_sent=True,sent_monotonic_wall=time.monotonic(),sent_UTC=now())
(B/'EXTERNAL_OWNER_PHYSICS_STOP_SENT.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'signal_sent':True,'owned_pgid':saved['pgid'],'run':str(RUN),'original_runner_remains_owner_of_cleanup':True}))
