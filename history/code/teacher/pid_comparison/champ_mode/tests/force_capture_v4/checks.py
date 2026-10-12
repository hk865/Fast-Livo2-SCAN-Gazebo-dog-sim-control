#!/usr/bin/env python3
"""Offline compiled cache, reader safety, SDF/ELF, preservation and readonly audit."""
from pathlib import Path
import copy
import hashlib
import importlib.util
import json
import subprocess
import xml.etree.ElementTree as ET
OUT=Path(__file__).resolve().parent;HERE=OUT.parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def invoke(args):return subprocess.run(args,capture_output=True,text=True,check=True)
checks={};evidence={}
binary=OUT/'cache_checks'
invoke(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(OUT/'cache_checks.cpp'),'-o',str(binary)])
evidence['compiled_cache']=invoke([str(binary)]).stdout
checks['compiled_actual_cache_clear_missing_true_zero_no_clamp_phase_cases']=True
spec=importlib.util.spec_from_file_location('champ_v4_reader',HERE/'command_file_reader.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
state={'world_sim_time':5.,'physics_iteration':1000,'force_capture_iteration':1000,'force_capture_sim_time':5.,'force_capture_dt':.005,
 'torque_source':'command_feed_to_physics','force_capture_phase':'PreUpdate after CM write, before Physics Update','force_capture_pair_valid':True,
 'force_command_valid':True,'tau':[2.]*12,'tau_available':[1]*12,'qd':[1.]*12,'actuator_limit_violation_latched':None}
checks['valid_same_step_force_accepted']=m.actuator_safety_reason(state,5.)is None
for label,changes,expected in [
 ('force_over_limit',{'tau':[23.50001]+[0.]*11},'force_command_limit_exceeded'),
 ('velocity_over_limit',{'qd':[30.00001]+[0.]*11},'joint_velocity_limit_exceeded'),
 ('missing_force',{'tau':[None]*12,'force_command_valid':False},'force_capture_unavailable'),
 ('same_step_mismatch',{'force_capture_iteration':999},'force_capture_iteration_mismatch'),
 ('sim_time_mismatch',{'force_capture_sim_time':4.995},'force_capture_time_mismatch'),
 ('dt_mismatch',{'force_capture_dt':.004},'force_capture_time_mismatch'),
 ('wrong_phase',{'force_capture_phase':'PostUpdate'},'force_capture_source_invalid'),
 ('false_pair',{'force_capture_pair_valid':False},'force_capture_unavailable'),
 ('partial_values',{'tau':[None]+[1.]*11},'force_capture_values_invalid'),
 ('partial_available',{'tau_available':[0]+[1]*11},'force_capture_values_invalid'),
 ('nonfinite_force',{'tau':[float('nan')]+[1.]*11},'force_capture_values_invalid'),
 ('latched_200Hz_exceedance',{'actuator_limit_violation_latched':'force_command_limit_exceeded'},'force_command_limit_exceeded')]:
 candidate={**state,**changes};before=copy.deepcopy(candidate)
 result=m.actuator_safety_reason(candidate,5.)
 checks[label+'_failed_closed']=result==expected
 # NaN identity comparison is intentional via encoded data; no clamping occurs.
 checks[label+'_raw_untouched']=json.dumps(candidate,sort_keys=True)==json.dumps(before,sort_keys=True)
checks['startup_missing_allowed_until_3']=m.actuator_safety_reason({**state,'force_command_valid':False,'tau':[None]*12},2.9)is None
checks['actual_boundary_23_5_and_30_not_rejected']=m.actuator_safety_reason({**state,'tau':[23.5]*12,'qd':[30.]*12},5.)is None
cpp=(HERE/'native/champ_native_observer.cpp').read_text()
checks['native_no_actuation_API']=all(x not in cpp for x in ['.SetForce(','.ResetPosition(','.ResetVelocity(','.SetLinearVelocity(','.SetAngularVelocity(','.AddWorldWrench('])
pre=cpp.split('void PreUpdate(',1)[1].split('void PostUpdate(',1)[0]
checks['PreUpdate_const_ECM_only_reads']='static_cast<const gz::sim::EntityComponentManager &>'in pre and '.CreateComponent('not in pre and '.SetData('not in pre
checks['ABI_registration_PreUpdate_and_priority']='ISystemPreUpdate)'not in cpp and 'NativeObserver::ISystemPreUpdate'in cpp and 'NativeObserver::ISystemConfigurePriority'in cpp
checks['priority_configure_1']='ConfigurePriority() override { return 1; }'in cpp
checks['missing_force_JSON_null']='else out << "null"'in cpp
checks['state_PostUpdate_offset0']='\\\"state_time_offset_s\\\":0'in cpp
checks['200Hz_violation_latched_no_clamp']='actuatorViolation.empty()'in cpp and 'std::abs(*tau[i])>23.5+1e-6'in cpp
elf=invoke(['nm','-D','-C',str(HERE/'native/build/libchamp_native_observer.so')]).stdout
checks['ELF_exports_PreUpdate_and_priority']='NativeObserver::PreUpdate'in elf and 'NativeObserver::ConfigurePriority'in elf
checks['ELF_has_no_force_or_pose_writer_symbols']=all(x not in elf for x in ['::SetForce(','::ResetPosition(','::ResetVelocity(','::AddWorldWrench('])
run=OUT/'prepared_offline';contract=json.loads((run/'champ_contract.json').read_text())
checks['contract_source_header_frozen']=contract['executor_source_sha256'].get(str(HERE/'native/force_capture.hh'))==sha(HERE/'native/force_capture.hh')
checks['all_current_CHAMP_source_refs_exact']=all(sha(p)==h for p,h in contract['executor_source_sha256'].items())
checks['all_run_artifacts_exact']=all(sha(p)==h for p,h in contract['artifacts'].items())
checks['14_original_shared_camera_assets_protected']=len(contract['protected_before'])==14 and all(sha(p)==h for p,h in contract['protected_before'].items())
world=ET.parse(run/'world.sdf').getroot();observer=world.find("world/model[@name='go2']/plugin[@name='champ_compare::NativeObserver']")
checks['priority_XML1_parses']=observer.find('{https://gazebosim.org/sdf}system_priority').text=='1'
old=ET.parse(run/'champ/input_teacher_world.sdf').getroot()
for tree in (old,world):
 model=tree.find("world/model[@name='go2']")
 for plugin in list(model.findall('plugin')):
  if plugin.get('name')in ['teacher_sim::TeacherActuator','gz_ros2_control::GazeboSimROS2ControlPlugin','champ_compare::NativeObserver']:model.remove(plugin)
checks['scene_physics_pose_collision_sensors_exact_XML']=ET.tostring(old)==ET.tostring(world)
sdf=invoke(['gz','sdf','-k',str(run/'world.sdf')]);evidence['SDF_check']={'stdout':sdf.stdout,'stderr':sdf.stderr};checks['SDF_parser_valid_no_sim_start']='Valid.'in sdf.stdout
resolution=json.loads((run/'champ/loader_resolution.json').read_text());checks['11_ELF_ldd_no_notfound']=len(resolution['libraries'])==11 and all(v['returncode']==0 and'not found'not in v['stdout']for v in resolution['libraries'].values())
preserved=json.loads((OUT/'legacy_v3/preservation_receipt.json').read_text());checks['all_874_dd50_existing_raw_files_unchanged']=all(sha(p)==h for p,h in preserved['sha256'].items())
checks['old_V3_native_DSO_preserved']=sha(OUT/'legacy_v3/native/build/libchamp_native_observer.so')=='8dc70ac7b96f8156c8a3d138774442d2c9eeab5d76ee7628ec762f75e2636ee7'
checks['baseline_launch_unchanged']=sha(HERE/'baseline.launch.py')==sha(OUT/'legacy_v3/baseline.launch.py') if (OUT/'legacy_v3/baseline.launch.py').exists() else sha(HERE/'baseline.launch.py')=='fc1f89fbfabe3120509dab046d477bb1c7a1697e6ff044d1efcf9ddea4059695'
checks['CHAMP_PD_and_initialization_unchanged']=contract['pid']['p']==220.982919 and contract['pid']['d']==1 and '(0,.9,-1.8 per leg)'in contract['initialization']
checks['command_consumer_AST_unchanged']=m.consumer()[1]=='eb42ee6044a279b1d0d44452f94a17dc6afc472f059c32be6510afae8c5a9ac0'
report={'schema':1,'scope':'CHAMP V4 passive before-physics command capture and safety only; no ROS/GZ/training execution','checks':checks,
 'passed':all(checks.values()),'passed_count':sum(checks.values()),'total':len(checks),'evidence':evidence,'actual_force_limit_pass':False,
 'source_sha256':{str(p):sha(p)for p in [HERE/'prepare.py',HERE/'command_file_reader.py',HERE/'native/champ_native_observer.cpp',HERE/'native/force_capture.hh',Path(__file__),OUT/'cache_checks.cpp']},
 'native_so_sha256':sha(HERE/'native/build/libchamp_native_observer.so')}
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'passed':report['passed'],'counts':[report['passed_count'],report['total']],'failed':[k for k,v in checks.items()if not v]}))
raise SystemExit(0 if report['passed']else 1)
