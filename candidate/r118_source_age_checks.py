"""Finite real bridge/file/reader business checks. Starts no ROS node or model."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent


def child(out):
    import bridge
    import controller  # Real PID/Teacher/message types, no Node construction.
    import executor_command
    import r118_source_age_probe as probe
    from types import SimpleNamespace as NS
    from unittest.mock import patch
    checks, outcomes = {}, []
    with tempfile.TemporaryDirectory(prefix='r118_clock_', dir=out) as tmp:
        run = Path(tmp)
        helper = Path(probe.__file__).resolve()
        (run/'navigation_scope.json').write_text(json.dumps({'references':{
            str(helper):hashlib.sha256(helper.read_bytes()).hexdigest()}}))
        if probe.ENABLED:
            valid_scope=(run/'navigation_scope.json').read_text()
            (run/'navigation_scope.json').write_text(json.dumps({'references':{}}))
            try:probe.bind('reader',run)
            except RuntimeError:pass
            else:raise AssertionError('Unfrozen helper was accepted')
            assert probe.PROBE is None
            (run/'navigation_scope.json').write_text(valid_scope)
            checks['unfrozen_helper_binding_refused_before_business']=True
        binding = probe.bind('reader',run)
        checks['selected_binding'] = (binding is not None) == probe.ENABLED
        source = dict(schema='actual_SLAM_SCAN_cascade_command_source/v1',
            cascade_sequence=12,source_pose_stamp_ns=27_625_000_000,
            paired_imu_stamp_ns=27_615_000_000,control_stamp_ns=27_790_000_000,
            command_after_slew=[.2,0.,0.],navigation_ground_truth_used=False,
            source_pose_received_wall_ns=99_990_000_000,source_imu_received_wall_ns=99_980_000_000,
            source_pose_received_monotonic_wall=99.99,paired_imu_received_monotonic_wall=99.98)
        base = dict(schema_version=1,mode='scan_slam',source='scan_slam',
            acceptance={'sha256':'scope'},sequence=7,command=[.2,0.,0.],
            monotonic_wall=100.,sim_time=27.9,controller_source=source,
            healthy=True,stop_requested=False,stamp={'ros_sim_time_ns':27_900_000_000})
        writer=bridge.CommandFile(run/'navigation_command.json')
        try:
            # Same original source passes bridge at age275ms, later fails reader
            # at age305ms although producer is only30ms and control140ms old.
            bridge.validate_controller_source(source,[.2,0.,0.],27_900_000_000,100_000_000_000)
            writer.write(base)
            cases=[('age300_accepted',base,27_925_000_000,False),
                   ('age305_rejected',base,27_930_000_000,True),
                   ('age320_rejected',base,27_945_000_000,True)]
            for name,changes,readclock,rejected in cases:
                raw=(run/'navigation_command.json').read_bytes()
                reader=executor_command.CommandReader(run.name,'scope',[.6,.2,.3])
                command,bad,record=reader.read(raw,readclock,100.01)
                assert bad is rejected,(name,record)
                if rejected:
                    assert record['reason']=='ValueError: Source age/causality rejected'
                    assert list(command)==[0.,0.,0.] and reader.frame is None and reader.authorization is None
                outcomes.append((name,list(command),bad,record['reason']))
            for name,key,value,clock in (
                ('future_pose','source_pose_stamp_ns',27_940_000_000,27_930_000_000),
                ('imu_gap','paired_imu_stamp_ns',27_600_000_000,27_900_000_000),
                ('control_before_pose','control_stamp_ns',27_620_000_000,27_900_000_000),
                ('future_control','control_stamp_ns',28_000_000_000,27_900_000_000)):
                data=copy.deepcopy(base);data['controller_source'][key]=value
                writer.write(data)
                reader=executor_command.CommandReader(run.name,'scope',[.6,.2,.3])
                cmd,bad,record=reader.read((run/'navigation_command.json').read_bytes(),clock,100.01)
                assert bad and record['reason']=='ValueError: Source age/causality rejected',name
                outcomes.append((name,list(cmd),bad,record['reason']))
            checks['real_bridge275_file_reader300_305_320_and_causality'] = True
            for name in ('oversize_command', 'oversize_integer'):
                malformed=copy.deepcopy(base)
                if name=='oversize_command':malformed['command']=[0]*10000
                else:malformed['controller_source']['source_pose_stamp_ns']=1<<128
                reader=executor_command.CommandReader(run.name,'scope',[.6,.2,.3])
                cmd,bad,record=reader.read(json.dumps(malformed).encode(),27_930_000_000,100.01)
                assert bad and list(cmd)==[0.,0.,0.]
                outcomes.append((name,list(cmd),bad,record['reason']))
            checks['malformed_container_and_integer_probe_work_bounded']=True
            # Diagnostics must not rewrite/retain the source payload.
            data=copy.deepcopy(base);writer.write(data)
            before=(run/'navigation_command.json').read_bytes()
            reader=executor_command.CommandReader(run.name,'scope',[.6,.2,.3])
            if probe.ENABLED:
                with patch.object(probe.PROBE,'note',side_effect=RuntimeError('diagnostic failure')):
                    cmd,bad,record=reader.read(before,27_925_000_000,100.01)
                assert not bad
                outcomes.append(('observer_failure',list(cmd),bad,record['reason']))
                # Actual adapter accepts scalar row from controller; failing
                # observational now never modifies caller state or clock.
                state=NS(command=[.2,0.,0.],state='running',get_clock=lambda:NS(now=lambda:(_ for _ in ()).throw(RuntimeError('clock sample'))))
                probe.controller(state,'select.row_complete',27_790_000_000,
                    dict(sequence=12,source_pose_stamp_ns=27_625_000_000,paired_imu_stamp_ns=27_615_000_000,
                         control_stamp_ns=27_790_000_000,feedback={'received_wall_ns':99_990_000_000}))
                assert state.command==[.2,0.,0.] and state.state=='running'
                checks['observer_failure_business_unchanged']=True
            else:
                cmd,bad,record=reader.read(before,27_925_000_000,100.01)
                outcomes.append(('observer_failure',list(cmd),bad,record['reason']))
            assert before==(run/'navigation_command.json').read_bytes() and data==base
            checks['source_bytes_and_object_unchanged']=True
            # Finite representative actual reader loop measures probe overhead
            # and proves windows stay bounded beyond evictions.
            reader=executor_command.CommandReader(run.name,'scope',[.6,.2,.3])
            wall0,cpu0=time.monotonic_ns(),time.process_time_ns()
            for _ in range(6000):
                cmd,bad,record=reader.read(before,27_930_000_000,100.01)
                assert bad
            cost=dict(calls=6000,wall_ns=time.monotonic_ns()-wall0,cpu_ns=time.process_time_ns()-cpu0)
        finally:writer.close()
        if probe.ENABLED:
            receipt=probe.PROBE.receipt()
            assert len(receipt['tail'])==probe.CAPACITY
            assert receipt['counts']['source_rejected']>=6007
            assert receipt['counts']['evicted']>0
            assert receipt['windows']['first_source_rejected']['trigger'][14]==305_000_000
            assert all(len(w['prior'])<=probe.CAPACITY and len(w['post'])<=probe.POST for w in receipt['windows'].values())
            for row in receipt['tail']:
                assert len(row)==len(probe.COLUMNS) and all(x is None or type(x) in (str,int,bool) for x in row)
                assert all(-(1<<63)<=x<(1<<63) for x in row if type(x)is int)
            checks['actual305_rejection_trigger_and_bounded_scalar_windows']=True
            checks['injected_errors_counted']=receipt['counts']['errors']==2
            probe.finish(run)
            emitted=run/'r118_source_age_reader.json'
            assert emitted.stat().st_size<1024*1024
            receipt=json.loads(emitted.read_text())
            checks['exit_only_receipt_bytes']=emitted.stat().st_size
        else:
            assert probe.PROBE is None
            receipt=None
    value=dict(selected=probe.ENABLED,checks=checks,outcomes=outcomes,reader_cost=cost,receipt=receipt,
        imports='real controller/Teacher/bridge/worker ROS types; no Node/Context/model',
        software_component_PASS=all(v is not False for v in checks.values()),ROS_or_navigation_PASS=False)
    (out/('ON.json' if probe.ENABLED else 'OFF.json')).write_text(json.dumps(value,allow_nan=False)+'\n')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--child',action='store_true');a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    if a.child:return child(a.out)
    for selected in ('0','1'):
        env=dict(os.environ,R118_SOURCE_AGE_PROBE=selected,PYTHONDONTWRITEBYTECODE='1')
        subprocess.run([sys.executable,str(Path(__file__).resolve()),'--out',str(a.out),'--child'],env=env,check=True)
    off,on=(json.loads((a.out/(mode+'.json')).read_text())for mode in ('OFF','ON'))
    assert off['outcomes']==on['outcomes']
    result=dict(software_component_PASS=True,business_ON_OFF_equal=True,ROS_or_navigation_PASS=False,
        off_cost=off['reader_cost'],on_cost=on['reader_cost'],
        additional_cpu_ns_per_read=(on['reader_cost']['cpu_ns']-off['reader_cost']['cpu_ns'])/6000,
        limitations='no ROS scheduling, Actor or full PID math timing; real run required for attribution',
        sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (HERE/'r118_source_age_probe.py',HERE/'controller.py',HERE/'bridge.py',HERE/'executor_command.py',HERE/'worker.py')})
    (a.out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
