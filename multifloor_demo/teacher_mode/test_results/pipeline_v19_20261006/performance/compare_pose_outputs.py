#!/usr/bin/env python3
"""Compare exact shared odometry acquisition stamps, without truth/alignment."""
import argparse
import csv
import json
import math
from pathlib import Path
from read_stages import sha


def poses(run):
    rows=[json.loads(x) for x in (run/'shadow_poses.jsonl').read_text().splitlines()]
    if len({r['source_ns'] for r in rows})!=len(rows):raise ValueError('Duplicate odometry source header cannot be silently overwritten')
    for r in rows:
        if len(r['position'])!=3 or len(r['quaternion_xyzw'])!=4 or not all(math.isfinite(v) for v in r['position']+r['quaternion_xyzw']):raise ValueError('Invalid pose')
        if sum(x*x for x in r['quaternion_xyzw'])<=0:raise ValueError('Invalid quaternion norm')
    return {r['source_ns']:r for r in rows}


def q_angle(a,b):
    an=math.sqrt(sum(x*x for x in a));bn=math.sqrt(sum(x*x for x in b))
    dot=abs(sum((x/an)*(y/bn) for x,y in zip(a,b)))
    return 2*math.acos(max(-1,min(1,dot)))


def iteration_evidence(run):
    counts={100:0,201:0};per_iteration={100:{},201:{}}
    with (run/'fastlivo_diagnostics/index.csv').open()as f:
        for row in csv.DictReader(f):
            kind=int(row['kind']);it=int(row['iteration'])
            if kind in counts and it<(1<<63):
                counts[kind]+=1;per_iteration[kind][str(it)]=per_iteration[kind].get(str(it),0)+1
    s=json.loads((run/'DESCRIPTIVE_SUMMARY.json').read_text());b=s['boundary']['boundaries']
    return {'kind100_LIO_iteration_rows':counts[100],'kind201_VIO_iteration_rows':counts[201],
        'per_iteration_rows':per_iteration,'per_frame_iteration_counts':'UNVERIFIED' if not counts[100] or not counts[201] else 'requires explicit frame grouping',
        'why':'Detail-only kind100/201 not enabled within captured0–60s; configured115–118s. Absence is not zero iterations.',
        'LIO_query_calls_completed_boundary_windows':b['LIO_residual_query']['calls_in_completed_windows'],
        'StateEst_calls_completed_boundary_windows':b['LIO_StateEst']['calls_in_completed_windows'],
        'partial_mean_LIO_iterations_per_StateEst':b['LIO_residual_query']['calls_in_completed_windows']/b['LIO_StateEst']['calls_in_completed_windows'] if b['LIO_StateEst']['calls_in_completed_windows'] else None,
        'partial_LIO_definition':'One BuildResidualListOMP call per StateEst loop iteration per production voxel_map.cpp; completed1s windows only, final partial window omitted. Not per-frame counts.',
        'VIO_iteration_count':'UNVERIFIED; processFrame calls do not identify pyramid iteration count'}


def compare(reference, runs):
    reference=reference.resolve();ref=poses(reference);rows=[]
    for run in runs:
        run=run.resolve();current=poses(run);shared=sorted(ref.keys()&current.keys())
        if not shared:raise ValueError('No exact shared source stamp')
        deltas=[];angles=[]
        for stamp in shared:
            a,b=ref[stamp],current[stamp]
            if (a['frame_id'],a['child_frame_id'])!=(b['frame_id'],b['child_frame_id']):raise ValueError('Different frame identity')
            deltas.append(math.sqrt(sum((x-y)**2 for x,y in zip(a['position'],b['position']))))
            angles.append(q_angle(a['quaternion_xyzw'],b['quaternion_xyzw']))
        rows.append({'run':str(run),'matched_exact_header_count':len(shared),'reference_count':len(ref),'run_count':len(current),
            'only_reference_stamps':sorted(ref.keys()-current.keys()),'only_run_stamps':sorted(current.keys()-ref.keys()),
            'first_shared_ns':shared[0],'last_shared_ns':shared[-1],
            'position_difference_m':{'RMS':math.sqrt(sum(x*x for x in deltas)/len(deltas)),'maximum':max(deltas),'last':deltas[-1]},
            'normalized_quaternion_angle_difference_deg':{'RMS':math.degrees(math.sqrt(sum(x*x for x in angles)/len(angles))),'maximum':math.degrees(max(angles)),'last':math.degrees(angles[-1])},
            'iteration_evidence':iteration_evidence(run),'source_sha256':{str(run/name):sha(run/name) for name in ('shadow_poses.jsonl','PLAN.json','DESCRIPTIVE_SUMMARY.json','fastlivo_diagnostics/index.csv')}})
    return {'schema':'V19_exact_header_odometry_output_difference/v1','reference_run':str(reference),
        'reference_iterations':iteration_evidence(reference),'rows':rows,'truth_used':False,'SE3_or_initial_pose_alignment_applied':False,
        'configured_common_coordinate_frame':True,'empirical_physical_truth_accuracy':'UNVERIFIED',
        'same_frontend_output_byte_exact_claimed':False,'different_callback_order_can_change_sync_slices_maps_and_iteration_paths':True,
        'comparison_not_accuracy_or_fullmath_PASS':True,'reader_sha256':sha(Path(__file__))}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',type=Path,required=True);p.add_argument('--run',type=Path,nargs='+',required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    with args.out.open('x')as f:json.dump(compare(args.reference,args.run),f,indent=2,allow_nan=False);f.write('\n')


if __name__=='__main__':main()
