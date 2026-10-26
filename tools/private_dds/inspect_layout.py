#!/usr/bin/env python3
"""Exact ELF identity/symbol/DWARF/source-call inspection; no inferior is run."""
import argparse,hashlib,json,os,re,subprocess
from pathlib import Path

def command(args):return subprocess.run(args,check=True,capture_output=True,text=True,env={**os.environ,'DEBUGINFOD_URLS':''}).stdout
p=argparse.ArgumentParser(description=__doc__);p.add_argument('elf',type=Path);p.add_argument('--out',type=Path,required=True);p.add_argument('--expected-sha256');p.add_argument('--expected-build-id');p.add_argument('--probe-source',type=Path,required=True);a=p.parse_args();elf=a.elf.resolve()
sha=hashlib.file_digest(elf.open('rb'),'sha256').hexdigest()
if a.expected_sha256 and sha!=a.expected_sha256:raise SystemExit('ELF SHA mismatch')
notes=command(['readelf','-n',str(elf)]);bid=re.search(r'Build ID: (\w+)',notes).group(1)
if a.expected_build_id and bid!=a.expected_build_id:raise SystemExit('ELF BuildID mismatch')
macros={k:v for k,v in re.findall(r'^#define (SHMSEND|ACK2|SYNC|GETBUF) "([^"]+)"',a.probe_source.read_text(),re.M)}
syms={}
for l in command(['nm','-D','--defined-only',str(elf)]).splitlines():
    parts=l.split()
    if len(parts)<3:continue
    name=parts[2].split('@@')[0]
    for role,symbol in macros.items():
        if name==symbol:syms[role]=dict(mangled=symbol,ELF_offset=int(parts[0],16),hex=hex(int(parts[0],16)),type=parts[1])
if set(syms)!=set(macros) or len(syms)!=4:raise SystemExit('required exact hook symbols missing')
script=Path(__file__).with_name('dwarf_layout.gdb.py')
gdb=command(['gdb','-nx','-batch','-iex','set debuginfod enabled off','-iex','set auto-load python-scripts off',str(elf),'-ex','set language c++','-ex','source '+str(script)])
raw=next(l.split('=',1)[1]for l in gdb.splitlines()if l.startswith('PROBE_LAYOUT_JSON='));dw=json.loads(raw)
T=dw['types'];ns='eprosima::fastrtps::rtps::'
def off(t,m):return next(f['offset']for f in T[t]['fields']if f['name']==m)
up=dw['unique_ptr_raw_pointer_paths'];sp=dw['cv_shared_ptr_raw_pointer_paths']
assert len(up)==len(sp)==1
# Exact base offsets must make StatefulReader->RTPSReader->Endpoint zero.
for t,b in [('StatefulReader','RTPSReader'),('RTPSReader','Endpoint')]:
    assert any(f['base'] and f['name']==ns+b and f['offset']==0 for f in T[ns+t]['fields'])
layout=dict(reader_participant_pointer_offset=off(ns+'Endpoint','mp_RTPSParticipant'),reader_mutex_A_offset=off(ns+'Endpoint','mp_mutex'),participant_manager_pointer_offset=off(ns+'RTPSParticipantImpl','send_buffers_')+up[0]['offset'],manager_mutex_M_offset=off(ns+'SendBuffersManager','mutex_'),manager_I_pointer_offset=off(ns+'SendBuffersManager','available_cv_')+off('eprosima::fastrtps::TimedConditionVariable','_M_mutex')+sp[0]['offset'],participant_mutex_S_offset=off(ns+'RTPSParticipantImpl','m_send_resources_mutex_'))
for role,paths in dw['native_lock_paths'].items():assert len(paths)==1 and paths[0]['offset']==0,(role,paths)
healthy=[]
for l in command(['nm','-S',str(elf)]).splitlines():
    parts=l.split()
    if len(parts)==4 and parts[3].endswith('SharedMemGlobal4Port13healthy_checkEv'):
        healthy.append((int(parts[0],16),int(parts[1],16),parts[3]))
if len(healthy)!=1:raise SystemExit('one exact healthy_check symbol required')
start,size,symbol=healthy[0];dis=command(['objdump','-d','--start-address='+hex(start),'--stop-address='+hex(start+size),str(elf)])
lines=dis.splitlines();calls=[]
for i,l in enumerate(lines):
    if re.search(r'\bcall\s+.*<nanosleep@plt>',l):
        call=int(l.strip().split(':')[0],16);nxt=next(x for x in lines[i+1:]if re.match(r'^\s*[0-9a-f]+:',x));ret=int(nxt.strip().split(':')[0],16)
        source=command(['addr2line','-i','-f','-C','-e',str(elf),hex(call)])
        calls.append(dict(call=call,return_PC=ret,call_hex=hex(call),return_PC_hex=hex(ret),source_inlining=source.splitlines(),instruction=l.strip()))
report=dict(schema='private_DDS_probe_ELF_layout/v1',file_only_no_inferior=True,elf=dict(path=str(elf),sha256=sha,build_id=bid),symbols=syms,layout=layout,layout_hex={k:hex(v)for k,v in layout.items()},DWARF=dw,health=dict(symbol=symbol,begin=start,size=size,nanosleep_calls=calls,exact_sleep_coverage=bool(calls)),probe_source=dict(path=str(a.probe_source.resolve()),sha256=hashlib.file_digest(a.probe_source.open('rb'),'sha256').hexdigest()),limits=['Member/native-lock layouts are proved for this exact ELF/config; do not transfer to another BuildID without rerunning.','I is a pointer to the condition_variable_any internal mutex; not the condition futex itself.','A/M/S/I native first __lock offset0 is supported by exact std-library DWARF; no live object or lock holder is sampled.','Exact healthy nanosleep callsites may disappear/change with patch; empty means hook unsupported, not absence of runtime waits.'])
a.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(out=str(a.out),elf_sha=sha,build_id=bid,layout=report['layout_hex'],symbols={k:v['hex']for k,v in syms.items()},health_return_PCs=[x['return_PC_hex']for x in calls])))
