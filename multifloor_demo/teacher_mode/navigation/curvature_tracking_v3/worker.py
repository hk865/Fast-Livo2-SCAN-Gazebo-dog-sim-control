#!/usr/bin/env python3
"""Isolated adapter around an archived unchanged CPU Teacher worker."""
import argparse
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--socket',required=True)
    args=parser.parse_args();run=args.run.resolve();p=json.loads((run/'truth_profile.json').read_text())
    sys.path.insert(0,str(run/'sources/policy'));sys.path.insert(0,str(run/'sources/truth'))
    from core import Controller
    spec=importlib.util.spec_from_file_location('frozen_teacher_worker',run/'sources/policy/worker.py')
    legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
    controller=Controller(p);capture={'state':None,'metadata':None};old_recv=legacy.recv_exact;old_atomic=legacy.atomic_json
    log=(run/'control.jsonl').open('w',buffering=1)
    def receive(conn,n):
        data=old_recv(conn,n)
        if n==512:capture['state']=np.frombuffer(data,dtype='<f8').copy()
        return data
    def command(test,t):
        cmd,phase=controller.update(capture['state'],t)
        log.write(json.dumps(controller.row,allow_nan=False)+'\n')
        if controller.completed_t is not None and capture['metadata']is not None:
            capture['metadata']['test_duration_s']=min(p['duration_s'],controller.completed_t+p['post_completion_s'])
        return cmd, 'initializing'if t<.1 else 'tracking'if np.linalg.norm(cmd)>1e-9 else'stop_transition'
    def write(path,obj):
        if path.name=='policy_manifest.json':
            obj.update(test='truth_pid_calibration',command_source='Simulation truth curve PID and prospective active pose hold v3 numerical binding',
                navigation_truth_used=True,counts_as_SLAM_navigation=False,
                test_duration_s=p['duration_s'],command_end_s=None,command_start_seconds=3.,
                feedback_source='native physics state, correctly time shifted world_time-.005',
                parking_semantics='fixed prospective goal active pose velocity hold v3 numerical binding; not original zero-velocity-only parking',
                truth_profile=p,legacy_motion_evaluation_applicable=False)
            capture['metadata']=obj
        if path.name=='worker_result.json':obj.update(truth_controller_completed_t_s=controller.completed_t,
            waypoint_events=controller.waypoint_events,counts_as_SLAM_navigation=False,
            termination_reason='truth_controller_completed'if controller.completed_t is not None else'duration_limit')
        old_atomic(path,obj)
    legacy.recv_exact=receive;legacy.requested=command;legacy.atomic_json=write;legacy.duration=lambda _:float(p['duration_s'])
    sys.argv=[str(run/'sources/policy/worker.py'),'--run',str(run),'--test','forward','--socket',args.socket]
    if p.get('terrain_target_manifest'):sys.argv+=['--terrain-target-manifest',str(run/'terrain_target_manifest.json')]
    try:legacy.main()
    finally:log.close()


if __name__=='__main__':main()
