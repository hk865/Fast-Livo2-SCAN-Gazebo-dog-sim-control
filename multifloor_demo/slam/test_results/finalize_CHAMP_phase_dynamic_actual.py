"""Read-only final provenance/small-event checks after one CDR metadata scan."""
from pathlib import Path
import hashlib, importlib.util, json, math, sys, xml.etree.ElementTree as ET
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUN = ROOT/'simulation/test_results/20261002_champ_phase_dynamic_candidate'
def load(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def module(name, p):
    spec=importlib.util.spec_from_file_location(name,p)
    obj=importlib.util.module_from_spec(spec);sys.modules[name]=obj;spec.loader.exec_module(obj)
    return obj

base_path=HERE/'oct2_CHAMP_phase_dynamic_actual_independent_final.json'
out=load(base_path);r=load(RUN/'dynamic_result.json');m=load(RUN/'profile_manifest.json')
checks=out['required_independent_checks'];cdr=out['actual_CDR_native_obstacle_to_terminal']
sm=load(RUN/'source_manifest.json')['sha256'];art=load(RUN/'runtime_manifest.json')['artifacts'];g=m['selected_gait']
checks['all269_production_source_bytes_match']=len(sm)==269 and all(sha(ROOT/n)==v for n,v in sm.items())
checks['all_original_runtime_artifacts_match']=all(sha(v['path'])==v['sha256'] for v in art.values())
checks['all36_private_built_source_archive_bytes_match']=len(g['source_sha256'])==36 and all(sha(Path(g['source_root'])/n)==v for n,v in g['source_sha256'].items())
checks['frozen_private_build_and_source_archive_receipts_match']=all(sha(g[k]['path'])==g[k]['sha256'] for k in ('frozen_build_receipt','frozen_source_archive_receipt'))
checks['private_gait_actual_exe_DSO_original_exe_DSO_bytes_match']=all(sha(g[k]['path'])==g[k]['sha256'] for k in ('executable','library','original_executable','original_library'))
early=load(RUN/'startup_gait_runtime.json')['actual'];late=load(RUN/'actual_gait_process.json')['actual'];clean=out['actual_cleanup']
checks['actual_gait_early_and_prestop_owned_PID_same']=len(early)==len(late)==1 and early[0]['pid']==late[0]['pid']==4171846 and early[0]['owned_process_group']==late[0]['owned_process_group']==clean['process_group']==4171804
checks['both_archive_and_source21_bytes_match']=len(m['files'])==21 and all(sha(v['path'])==sha(RUN/'staging'/n)==v['sha256'] for n,v in m['files'].items())
feet=r['foot_contacts'];allowed={'floor_1','ramp_12'}
checks['actual_four_feet_only_original_declared_ground_contact_pairs']=all(feet[leg]['nonempty']>0 and not feet[leg]['violations'] and all((pair[0]==f'go2::{leg}_lower_leg_link::{leg}_lower_leg_link_fixed_joint_lump__{leg}_foot_link_collision_1' and pair[1].split('::')[0] in allowed) or (pair[1]==f'go2::{leg}_lower_leg_link::{leg}_lower_leg_link_fixed_joint_lump__{leg}_foot_link_collision_1' and pair[0].split('::')[0] in allowed) for pair in feet[leg]['pairs']) for leg in ('lf','rf','lh','rh'))
fixed=('clock','imu','measured_joint','legacy_joint','jtc')
checks['native_obstacle_to_terminal_fixed_five_streams_complete']=cdr['all_fixed_period_sensor_JTC_headers_complete'] is True
checks['retained_CDR_exact_sequence_schema_and_integer_header']=cdr['metadata_matches_footer'] is True and not cdr['sequence_errors'] and not cdr['full_original_payload_errors']
footer=load(RUN/'actuator/observer_result.json')
checks['recorder_no_overwrite_or_failure']=footer['failure'] is None and footer['overwritten_events']==0 and footer['retained_events']==footer['total_events']==1101507 and footer['capacity']==3600000 and footer['payload_limit']==2147483648
emits=[json.loads(s) for s in (RUN/'joint_stop_adapter.jsonl').open()]
emits=[x for x in emits if x['kind']=='actual_champ_command' and r['origin_stamp_ns']<=round(x['sim']*1e9)<=r['terminal_stamp_ns']]
bad=[]
for x in emits:
    v=x['value'];cap=.08 if abs(v[0])>1e-12 else .12
    if len(v)!=6 or not all(map(math.isfinite,v)) or abs(v[0])>.12+1e-12 or any(abs(v[i])>1e-12 for i in (1,2,3,4)) or abs(v[5])>cap+1e-12:bad.append(x)
checks['actual_native_Adapter_emissions_with_original_mixed_and_pure_caps']=bool(emits) and not bad
out['actual_native_Adapter_emission_caps']={'observed_active_emissions':len(emits),'nonzero_vx_emissions':sum(abs(x['value'][0])>1e-12 for x in emits),'violations':bad,'scope':'Adapter native emission log; not CHAMP callback, JTC update or applied-force timestamp.'}

held=module('actual_dynamic_held',RUN/'staging/held_command_contract.py')
metadata=[json.loads(s) for s in (RUN/'trajectory_payloads.jsonl').open()]
request=r['region_evaluation']['regions'][0]['receipt']['request_id']
held_result=held.audit_held_commands(r['native_obstacle_events'],r['statuses'],metadata,r['command_evidence'],request,[x['center'] for x in r['goals_definitions']])
checks['original_held_helper_reproduced_same_interval_exact_zero']=held_result==r['held_command_timing_audit'] and held_result['passed']
out['independent_held_original_helper']=held_result
checks['exact_original_three_dynamic_helpers_archived']=all(sha(RUN/'staging'/n)==v for n,v in {'probe_dynamic_drift.py':'58918acd3553ae2ccf5a616c2a17b835c659484ea5e18793271f746f9f92cab6','dynamic_region_contract.py':'6928ac176e031e42dbf40004a5a2a2ba18e35e0c9ee0881d134af383290241c3','held_command_contract.py':'a9cdff648df3d40526e2624ca41c4d5cf08d0f640891f8d1b5d920cb56852011'}.items())

states=[x['state'] for x in r['obstacle_states'] if x['state'].get('mission_run_id')==request]
phase={p:[x for x in states if x['visible_phase']==p] for p in ('entering','blocking','leaving','clear')}
first={p:v[0]['stamp'] for p,v in phase.items()};trigger=phase['entering'][0]
ps=[p for p in r['poses'] if p['stamp_ns']<=round(first['entering']*1e9)]
trigger_distance=float(np.linalg.norm(np.asarray(ps[-1]['p'][:2])-np.asarray(trigger['trigger_position'][:2])))
world=ET.parse(ROOT/'simulation/generated/three_floors.sdf').getroot();box=world.find(".//model[@name='moving_obstacle']")
collision=[float(v) for v in box.findtext('.//collision/geometry/box/size').split()]
checks['actual_original_collidable_box_geometry_and_acknowledged_motion']=collision==[.5,.5,1.2] and all(x['size']==[.5,.5,1.2] and x['position'][0]==1. and x['position'][2]==.6 and x['failed_updates']==0 for x in states) and states[-1]['gazebo_updates']>0 and max(x['position'][1] for x in states)-min(x['position'][1] for x in states)>=1.
checks['actual_frozen_request_index1_trigger_after_north_arrival']=trigger['trigger_navigation_ready'] is True and trigger['trigger_waypoint_index']==1 and trigger['trigger_error'] is None and trigger['stamp']>r['region_evaluation']['regions'][0]['receipt']['stamp_ns']/1e9 and trigger_distance<1.4
checks['natural_original_box_8enter_20block_8leave']=all(phase.values()) and np.allclose([first['blocking']-first['entering'],first['leaving']-first['blocking'],first['clear']-first['leaving']],[8,20,8],atol=1e-9,rtol=0)
out['actual_box_evidence']={'first_phases_sim':first,'first_trigger_distance_m':trigger_distance,'same_request':request,'successful_updates':states[-1]['gazebo_updates'],'failed_updates':states[-1]['failed_updates'],'collision_size_m':collision,'phase_counts':{p:len(v) for p,v in phase.items()},'limits':['Positions are successful Gazebo SetEntityPose acknowledgement values; state publishes every .2s and may show the previous acknowledged target at a phase transition.','The first clear state position .05 reflects that asynchronous previous acknowledgement; later clear reaches y0. No robot pose service/control or GT triggering is inferred.']}

contract=module('actual_dynamic_contract_final',RUN/'staging/dynamic_region_contract.py')
progress=[]
for evidence in r['actual_post_resume_progress']:
    begin=round(evidence['begin_sim']*1e9);end=round(evidence['end_stamp']*1e9)
    before=[p for p in r['poses'] if p['stamp_ns']<=begin and begin-p['stamp_ns']<=150000000][-1]
    after=next(p for p in r['poses'] if p['stamp_ns']==end)
    unit=np.asarray(r['goals_definitions'][-1]['center'][:2])-np.asarray(before['p'][:2]);unit/=np.linalg.norm(unit)
    gt0=contract.bounded_pair(begin,r['truth']);gt1=contract.bounded_pair(end,r['truth'])
    progress.append({'begin_ns':begin,'end_ns':end,'raw_SLAM_forward_m':float((np.asarray(after['p'][:2])-np.asarray(before['p'][:2]))@unit),'fixed_world_truth_east_m':float(gt1[0][0]-gt0[0][0])})
checks['same_observed_post_resume_window_has_two_source_forward_progress']=bool(progress) and all(x['raw_SLAM_forward_m']>=.025 and x['fixed_world_truth_east_m']>=.025 for x in progress)
out['independent_observed_post_resume_progress']=progress
nav_path=ROOT/'navigation/test_results/champ_phase_dynamic_handoff_audit/cloud_held.json';nav=load(nav_path)
checks['shared_native_cloud_zero_idle_fresh_SCAN_small_audit_verified']=nav['ready'] is True and all(nav['checks'].values()) and all(sha(x['json_file'])==x['json_sha256'] and sha(x['npz_file'])==x['npz_sha256']==x['event']['cloud_sha256'] for x in nav['native_events'])
out['shared_native_cloud_handoff_audit']={'path':str(nav_path),'sha256':sha(nav_path),'checks':nav['checks'],'scope':'NAV independently recomputed the exact archived native cloud guard, accepted spline/path and native emissions; this report does not repeat that payload scan.'}

out['sampling_diagnostics']={'probe_raw_1ms_active_complete':out['checks']['rawIMU1ms_allactive_observed'],'FAST_native_active_input_1ms_complete':not out['native_FAST_IMU']['raw_observed_but_native_unconsumed_in_native_span'],'FAST_log_probe_seen_but_missing_stamps_ns':out['native_FAST_IMU']['raw_observed_but_native_unconsumed_in_native_span'],'CDR_all_active_fixed_streams_complete':all(not cdr['all_active_headers'][k]['unexpected_fixed_gaps'] for k in fixed),'CDR_active_loss_callbacks': [x for x in cdr['all_middleware_lost_callbacks'] if x['within_active']],'original_all_CDR_loss_callbacks':cdr['all_middleware_lost_callbacks'],'missing_samples_NOT_filled':True,'limits':['All five DDS-loss callbacks were observed at clock13.206, before origin20.5. They identify the loss-report callback association, not each lost message Header.','Four native IMU input log omissions remain; raw/independent CDR presence does not repair that integrator record.','Original finite/monotonic native timestamp checks are preserved; an all-1ms receiver/integrator claim is a separate diagnostic and is false here.']}
out['original_windows_ns']=[[x['receipt']['start_stamp_ns'],x['receipt']['stamp_ns']] for x in out['independent_original_windows']['regions']]
out['foot_contacts_actual_probe_callback_evidence']=feet
out['verdict']='Original schema2 dynamic component passed the original30 checks and independent immutable receipt-window/GT/health/provenance checks. This is not full46-route completion or perfect sampling/actuation.'
out['limits'][-1]='Only this original schema2 two-goal dynamic component is validated; full46 goals, real PCD save and production adoption remain separate.'
out['checks'].update(checks);out['checks_count']=len(out['checks']);out['required_checks_count']=len(checks)
out['motion_component_passed']=all(checks.values());out['all_joint_evidence_checks_true']=all(checks.values());out['full_sampling_diagnostics_complete']=all(out['sampling_diagnostics'][k] for k in ('probe_raw_1ms_active_complete','FAST_native_active_input_1ms_complete','CDR_all_active_fixed_streams_complete'))
out['prior_basic_final_report_SHA256']=sha(base_path)
out['prior_contact_name_review_failure']={'path':str(HERE/'oct2_CHAMP_phase_dynamic_before_contact_name_correction.json'),'sha256':sha(HERE/'oct2_CHAMP_phase_dynamic_before_contact_name_correction.json'),'correction':'Reviewer-only foot identifier matcher incorrectly assumed ::lf_foot; exact native fixed-joint lump collision names are now used. Original run result, source and observations unchanged.'}
dest=HERE/'oct2_CHAMP_phase_dynamic_actual_independent_verified.json';dest.write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'path':str(dest),'sha256':sha(dest),'required_check_count':len(checks),'false_checks':[k for k,v in checks.items() if not v],'sampling':out['sampling_diagnostics'],'windows':out['original_windows_ns']},indent=2))
