#!/usr/bin/env python3
"""Read an ended run. No ROS imports, node, command or GT control input."""
import hashlib
import json
import math
import pathlib
import sys
import bisect

HERE = pathlib.Path(__file__).resolve().parent
DEMO = HERE.parents[2]
RUN = DEMO/'runs/20261002_001559_882ddb'
sys.path.insert(0,str(DEMO/'navigation'))
from goal_regions import parse_goal, contains_control, definitions_sha256

def load_lines(path):
    rows=[];errors=[]
    for index,line in enumerate(path.read_text().splitlines()):
        try:rows.append(json.loads(line))
        except json.JSONDecodeError as ex:errors.append({'line':index+1,'error':str(ex)})
    return rows,errors

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def angular_error(target,yaw):return math.atan2(math.sin(target-yaw),math.cos(target-yaw))
def rpy(q):
    x,y,z,w=q;n=x*x+y*y+z*z+w*w
    return [math.atan2(2*(w*x+y*z),n-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)/n))),
            math.atan2(2*(w*z+x*y),n-2*(y*y+z*z))]

mission=json.loads((RUN/'mission.json').read_text())
feedback,ef=load_lines(RUN/'feedback_navigation.jsonl')
poses,ep=load_lines(RUN/'pose_audit.jsonl')
trajectories,et=load_lines(RUN/'feedback_navigation_trajectories.jsonl')
commands,ec=load_lines(RUN/'joint_stop_adapter.jsonl')
sensor,ei=load_lines(RUN/'sensor_audit.jsonl')
nav=mission['navigation']
goals=[parse_goal(x) for x in nav['goals_definitions']]
definition_hash=definitions_sha256(goals)
slam=sorted([x for x in poses if x['source']=='slam'],key=lambda x:x['stamp_ns'])
slam_by_ns={x['stamp_ns']:x for x in slam}

receipts=[]
for receipt in nav['region_arrivals']:
    goal=goals[receipt['waypoint_index']]
    start,end=receipt['start_stamp_ns'],receipt['stamp_ns']
    samples=[x for x in slam if start<=x['stamp_ns']<=end]
    previous=receipts[-1]['receipt']['stamp_ns'] if receipts else -1
    checks={
        'ordered_goal':goal.goal_id==receipt['goal_id'],
        'hash_matches':receipt['goals_definition_sha256']==definition_hash,
        'request_matches':receipt['request_id']==nav['request_id'],
        'full_dwell':end-start>=round(goal.dwell_sim_s*1e9),
        'strict_after_previous':start>previous,
        'exact_start_end_in_raw_audit':start in slam_by_ns and end in slam_by_ns,
        'all_raw_inner':bool(samples) and all(contains_control(goal,x['p']) for x in samples),
        'gap_bounded':bool(samples) and all(b['stamp_ns']-a['stamp_ns']<=200000000 for a,b in zip(samples,samples[1:])),
        'receipt_not_protected':receipt['protected'] is False,
        'receipt_declares_inner':receipt['control_region_inside'] is True,
        'receipt_geometry_matches':receipt['control_arrival_definition']==goal.control_arrival_definition(),
    }
    if end in slam_by_ns:
        checks['endpoint_raw_position_matches']=max(abs(a-b) for a,b in zip(receipt['raw_position'],slam_by_ns[end]['p']))<1e-12
    receipts.append({'receipt':receipt,'raw_samples':samples,'checks':checks,'all_checks':all(checks.values())})

metadata={}
for row in trajectories:
    if row['kind']=='scan_metadata':
        m=row['metadata'];metadata[(m['trajectory']['traj_id'],tuple(m['reference_stamp']))]=m
identity_checks=0;identity_problems=[];phase_edges=[];last_phase=None;capture_rows=[]
for row in feedback:
    n=row.get('navigation',{})
    if not n:continue
    k=(n.get('waypoint_index'),n.get('alignment_phase'),n.get('tilt_hold'),n.get('state'))
    if k!=last_phase:
        phase_edges.append({'sim_time':row.get('sim_time'),'wall_elapsed':row['wall_elapsed'],
                            'waypoint_index':k[0],'phase':k[1],'tilt_hold':k[2],'state':k[3],
                            'actuator_command':row.get('actuator_command'),
                            'raw_imu_tilt_rad':n.get('raw_imu_tilt_rad'),'message':n.get('message')})
        last_phase=k
    tid=n.get('accepted_trajectory_id');ref=n.get('trajectory_reference_stamp')
    if tid is not None and ref is not None and n.get('state')=='running':
        identity_checks+=1
        m=metadata.get((tid,tuple(ref)))
        goal=n.get('current_goal')
        if m is None or goal is None or max(abs(a-b) for a,b in zip(m['body_goal'],goal['center']))>1e-8:
            identity_problems.append({'sim_time':row.get('sim_time'),'traj_id':tid,'reference':ref,'current_goal':goal})
    if row.get('sim_time',0) and row['sim_time']>=111 and n.get('steering'):
        st=n['steering'];yaw=row.get('slam',{}).get('yaw');locked=n.get('locked_heading')
        capture_rows.append({'sim_time':row['sim_time'],'wall_elapsed':row['wall_elapsed'],
            'waypoint_index':n['waypoint_index'],'phase':n['alignment_phase'],
            'pose':row.get('slam',{}).get('pose'),'slam_yaw':yaw,
            'gt_yaw_independent_only':row.get('ground_truth',{}).get('yaw'),
            'locked_heading':locked,'locked_error':None if locked is None or yaw is None else angular_error(locked,yaw),
            'current_path_heading':st['heading'],'current_path_error_at_control_stamp':st['error'],
            'controller_steering_stamp':st['stamp'],'controller_odom_stamp':st['odom_stamp'],
            'actuator_command':row.get('actuator_command'),'raw_imu_tilt_rad':n.get('raw_imu_tilt_rad'),
            'trajectory_id':st.get('trajectory_id')})

raw=[]
for row in sensor:
    if row['source']!='imu' or row['stamp']<111:continue
    roll,pitch,yaw=rpy(row['quaternion'])
    raw.append({'stamp':row['stamp'],'wall_monotonic':row['wall_monotonic'],
                'tilt':max(abs(roll),abs(pitch)),'roll':roll,'pitch':pitch,'yaw':yaw,
                'angular_velocity':row['angular_velocity']})
first_hold=next(x for x in raw if x['tilt']>=.30)
first_fail=next(x for x in raw if x['tilt']>=.50)
max_tilt=max(raw,key=lambda x:x['tilt'])
events=[x for x in commands if x['kind'] in ['zero_edge','native_zero_ack','return_complete','failure','shutdown'] and x.get('sim',0)>=111]
actual=[x for x in commands if x['kind']=='actual_champ_command' and 112.8<=x['sim']<=first_fail['stamp']]
integral=0.;duration=0.;gaps=[]
for a,b in zip(actual,actual[1:]):
    dt=b['sim']-a['sim']
    if dt<0 or dt>.2:gaps.append([a['sim'],b['sim']]);continue
    duration+=dt;integral+=a['value'][5]*dt
all_post_hold_zero=all(max(map(abs,x['value']))==0 for x in actual if x['sim']>=first_hold['stamp'])
snapshots=[]
for wanted in [112,113,114,115,116,117,118,119,119.7,119.8,120]:
    snapshots.append(min(capture_rows,key=lambda x:abs(x['sim_time']-wanted)))
first_turn_slam=min(slam,key=lambda x:abs(x['stamp']-112.9))
hold_slam=min(slam,key=lambda x:abs(x['stamp']-119.8))
displacement=[b-a for a,b in zip(first_turn_slam['p'],hold_slam['p'])]

source_manifest=json.loads((RUN/'source_manifest.json').read_text())['sha256']
source_files=['navigation/controller.py','navigation/control_core.py','navigation/goal_regions.py',
              'navigation/trajectory_contract.py','simulation/control_bridge.py',
              'simulation/execution_safety.py','simulation/joint_reference_adapter.py']
source_checks={name:sha(DEMO/name)==source_manifest[name] for name in source_files}
input_files=['mission.json','source_manifest.json','runtime_manifest.json','feedback_navigation.jsonl',
             'pose_audit.jsonl','feedback_navigation_trajectories.jsonl','sensor_audit.jsonl','joint_stop_adapter.jsonl']
result={
 'run_id':RUN.name,'scope':'read-only native NAV/SLAM/command audit; GT yaw independent comparison only',
 'mission_terminal':{k:mission.get(k) for k in ['stage','message','process_running','stopping','elapsed_s','events']},
 'navigation_final':{k:nav.get(k) for k in ['state','waypoint_index','total','message','tilt_stops',
      'max_tilt_rad','raw_imu_max_tilt_rad','trajectory_association_rejected','degenerate_splines',
      'last_spline_rejected','obstacle_stops','replans','motion_limits']},
 'definition_hash':definition_hash,'hash_matches_mission':definition_hash==mission['current_goals_sha256'],
 'receipts':receipts,'all_receipt_raw_checks':all(x['all_checks'] for x in receipts),
 'phase_edges':phase_edges,'identity_checks':identity_checks,'identity_problems':identity_problems,
 'metadata_count':len(metadata),'accepted_path_count':sum(x['kind']=='accepted_path' for x in trajectories),
 'first_hold_actual_raw_imu':first_hold,'first_fail_actual_raw_imu':first_fail,
 'hold_to_fail_sim_s':first_fail['stamp']-first_hold['stamp'],'raw_observed_max':max_tilt,
 'last_segment_steering_snapshots':snapshots,
 'last_segment_locked_turn_slam_displacement':{'first':first_turn_slam,'last':hold_slam,
    'delta_position':displacement,'planar_norm':math.hypot(*displacement[:2])},
 'actual_adapter_command':{'range':[actual[0]['sim'],actual[-1]['sim']],
    'samples':len(actual),'sim_elapsed':duration,'yaw_command_integral':integral,
    'max_abs_yaw':max(abs(x['value'][5]) for x in actual),
    'max_abs_linear':max(max(map(abs,x['value'][:3])) for x in actual),
    'all_commands_zero_from_first_raw_hold':all_post_hold_zero,'timestamp_gaps':gaps,
    'semantics':'actual Adapter emit publication log; cached clock float, not independent CHAMP/JTC receive or body response'},
 'native_adapter_edges':events,
 'source_hash_checks':source_checks,
 'input_sha256':{name:sha(RUN/name) for name in input_files},
 'parse_errors':{'feedback':ef,'poses':ep,'trajectories':et,'commands':ec,'imu':ei},
 'limitations':['No GT control or new nodes',
    'No full-payload receive trace/JTC interval analysis here; Sim owns native publisher and physical boundary',
    'Cached status/steering can precede sampled pose: use embedded control stamp for exact current error',
    'Three raw-inner receipts are not a complete Demo PASS; independent outer GT dwell is reviewed separately',
    'Locked heading remains fixed during align by current policy; its difference from moving path direction alone is not proof of a bug']}
(HERE/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'output':str(HERE/'result.json'),'sha256':sha(HERE/'result.json'),
 'raw_receipts':len(receipts),'all_raw_checks':result['all_receipt_raw_checks'],
 'identity_checks':identity_checks,'identity_problems':len(identity_problems),
 'first_hold':first_hold['stamp'],'first_fail':first_fail['stamp'],'max_tilt':max_tilt['tilt'],
 'unchanged_source':all(source_checks.values()),'parse_errors':sum(map(len,[ef,ep,et,ec,ei]))}))
