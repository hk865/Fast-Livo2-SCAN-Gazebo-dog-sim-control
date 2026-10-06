"""Owned launch-time affinity only; never changes another task's processes."""
from pathlib import Path
import json, os, time
TASKSET='/usr/bin/taskset'
GROUPS={'gazebo':[0,1,2,3],'slam':[4,5,6,7],'actor':[18],'controller':[19],
        'communications':list(range(8,18))}
ROOT_ROLES={'worker':'actor','bridge':'communications','capture':'communications',
            'navigation_stack':'communications','gazebo':'gazebo'}
def parse_list(value):
    result=set()
    for token in value.strip().split(','):
        if not token:continue
        if '-' in token:
            a,b=map(int,token.split('-'));result.update(range(a,b+1))
        else:result.add(int(token))
    return sorted(result)
def topology():
    p=Path('/sys/bus/event_source/devices/cpu_core/cpus').read_text().strip()
    e=Path('/sys/bus/event_source/devices/cpu_atom/cpus').read_text().strip()
    return {'cpu_core_raw':p,'cpu_atom_raw':e,'P_cpus':parse_list(p),'E_cpus':parse_list(e),
            'online_cpus':parse_list(Path('/sys/devices/system/cpu/online').read_text()),
            'runner_allowed_cpus':sorted(os.sched_getaffinity(0)),
            'thread_siblings':{str(i):Path('/sys/devices/system/cpu/cpu'+str(i)+'/topology/thread_siblings_list').read_text().strip() for i in range(20)}}
def prepare_contract(run,profile):
    enabled=profile.get('cpu_affinity_enabled',False)
    if type(enabled)is not bool:raise RuntimeError('Explicit bool affinity profile required')
    data={'schema':'teacher_owned_cpu_affinity/v1','enabled':enabled,'simulation_only':True,
        'group_cpus':GROUPS,'root_role_group':ROOT_ROLES,'launch_child_exceptions':{'fastlivo_mapping':'slam','controller.py':'controller'},
        'other_owned_navigation_children':'communications','application':'taskset prefix on new owned subprocess only, children inherit explicit group; mapping and controller override at their own launch',
        'other_task_processes_changed':False,'runner_affinity_changed':False,
        'same_frozen_V12_core_and_controller_math':True,
        'isolation_limit':'CPU masks only. Does not reserve cores, move other tasks, or isolate GPU, memory bandwidth, disk, DDS scheduling or physical core ownership.',
        'expected_groups_disjoint':True,'topology':topology()}
    values=[set(v)for v in GROUPS.values()]
    assert all(not(a&b) for i,a in enumerate(values)for b in values[i+1:])
    if enabled:
        t=data['topology']
        if t['P_cpus']!=list(range(8)) or t['E_cpus']!=list(range(8,20)):
            raise RuntimeError('Frozen hybrid CPU grouping differs; explicit new audit needed')
        if not set.union(*values)<=set(t['online_cpus'])&set(t['runner_allowed_cpus']):
            raise RuntimeError('Requested CPU unavailable to owned launch')
        if any(t['thread_siblings'][str(i)]!=str(i)for i in range(20)):
            raise RuntimeError('Grouping assumes observed no-SMT CPU topology')
        if not Path(TASKSET).is_file():raise RuntimeError('Existing taskset missing')
    path=Path(run)/'cpu_affinity_contract.json'
    with path.open('x') as stream:json.dump(data,stream,indent=2);stream.write('\n')
    return data

def contract(run):
    d=json.loads((Path(run)/'cpu_affinity_contract.json').read_text())
    if d.get('schema')!='teacher_owned_cpu_affinity/v1' or d.get('group_cpus')!=GROUPS or d.get('root_role_group')!=ROOT_ROLES:
        raise RuntimeError('Frozen affinity contract mismatch')
    return d

def prefix(run,group):
    d=contract(run)
    if not d['enabled']:return []
    if group not in GROUPS:raise RuntimeError('Unknown owned CPU group')
    return [TASKSET,'--cpu-list',','.join(map(str,GROUPS[group]))]
def node_prefix(run,group):return ' '.join(prefix(run,group))
def command(run,role,argv):return prefix(run,ROOT_ROLES[role])+list(argv)
def await_root_mask(run,role,pid,poll):
    d=contract(run)
    if not d['enabled']:return
    expected=set(GROUPS[ROOT_ROLES[role]]);deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        if poll()is not None:raise RuntimeError('Owned affinity process exited before startup mask witness')
        try:
            if os.sched_getaffinity(pid)==expected:return
        except ProcessLookupError:pass
        time.sleep(.005)
    raise RuntimeError('Owned launch prefix did not establish expected CPU mask')

def expected_group(role,argv,here):
    if role=='navigation_stack':
        if any(Path(x).name=='fastlivo_mapping'for x in argv):return 'slam'
        if str(Path(here)/'controller.py')in argv:return 'controller'
        return 'communications'
    return ROOT_ROLES[role]
def witness(run,identities,group_members,here):
    d=contract(run)
    result={'schema':'teacher_owned_cpu_affinity_witness/v1','monotonic_wall_ns':time.monotonic_ns(),
        'enabled':d['enabled'],'only_saved_owned_process_groups':True,'groups':[],'verified':True}
    if not d['enabled']:return result
    seen=set()
    for role,saved in identities.items():
        if not saved:raise RuntimeError('Missing owned process identity for affinity witness')
        for process in group_members(saved['pgid']):
            pid=process['pid'];directory=Path('/proc')/str(pid)
            try:argv=[x.decode(errors='replace')for x in (directory/'cmdline').read_bytes().split(b'\0')if x]
            except FileNotFoundError:continue
            group=expected_group(role,argv,here);expected=GROUPS[group];seen.add(group);threads=[]
            try:tasks=list((directory/'task').iterdir())
            except FileNotFoundError:continue
            for task in tasks:
                try:
                    fields=dict(x.split(':',1)for x in (task/'status').read_text().splitlines()if ':'in x)
                    actual=parse_list(fields['Cpus_allowed_list'])
                except FileNotFoundError:continue
                row={'tid':int(task.name),'cpus_allowed_list':fields['Cpus_allowed_list'].strip(),'matches_expected':actual==expected}
                threads.append(row)
                if not row['matches_expected']:result['verified']=False
            result['groups'].append({'owned_root_role':role,**process,'argv':argv,'group':group,'expected_cpus':expected,'threads':threads})
    result['groups_seen']=sorted(seen)
    if set(seen)!=set(GROUPS):result['verified']=False
    with (Path(run)/'cpu_affinity_witness.jsonl').open('a')as stream:stream.write(json.dumps(result)+'\n')
    if not result['verified']:raise RuntimeError('An owned thread escaped its frozen CPU group; fail through existing owned-only cleanup')
    return result
