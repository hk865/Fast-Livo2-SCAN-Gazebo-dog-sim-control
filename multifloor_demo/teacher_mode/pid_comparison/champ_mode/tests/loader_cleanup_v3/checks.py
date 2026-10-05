#!/usr/bin/env python3
"""Resolve ELF only and mock shutdown/error receipt paths. No ROS initialized."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

OUT=Path(__file__).resolve().parent;HERE=OUT.parents[1]
spec=importlib.util.spec_from_file_location('champ_cleanup_current',HERE/'command_file_reader.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
checks={};evidence={}
class RCLError(Exception):pass
metadata={'controller_kind':'champ','completed':True,'fault':None,'raw_recorders_closed':True,'last_ros_sim_ns':188_000_000_000}
def fixture(name,context,publisher,input_metadata=None,expected_exception=None):
    with tempfile.TemporaryDirectory(prefix='champ_cleanup_')as temp:
        run=Path(temp);stage=run/'champ';stage.mkdir();raised=None
        try:m.finalize_receipts(run,stage,input_metadata or metadata,publisher,context,RCLError)
        except Exception as error:raised=error
        result=json.loads((run/'worker_result.json').read_text());policy=json.loads((run/'policy_metadata.json').read_text());done=json.loads((stage/'monitor_done.json').read_text())
        checks[name+'_all_receipts_equal']=result==policy==done
        checks[name+'_exception_boundary']=(raised is None if expected_exception is None else isinstance(raised,expected_exception))
        evidence[name]=result
        return result
calls=[];r=fixture('normal',lambda:True,lambda:calls.append('zero'))
checks['normal_zero_once_completed']=calls==['zero']and r['completed']is True and r['fault']is None
calls=[];r=fixture('closed_context',lambda:False,lambda:calls.append('zero'),{**metadata,'completed':False,'fault':'CHAMP monitor interrupted'})
checks['closed_context_no_publish_failed_preserved']=not calls and r['completed']is False and r['fault']=='CHAMP monitor interrupted'and bool(r['shutdown_publish_unavailable'])
def ros_error():raise RCLError('invalid context')
state=iter([True,False]);r=fixture('shutdown_race',lambda:next(state),ros_error,{**metadata,'completed':False,'fault':'CHAMP monitor interrupted'})
checks['shutdown_race_only_closed_RCLError_ignored']=r['completed']is False and r['final_zero_publish_attempted']is True and r['final_zero_publish_error']is None
r=fixture('live_ROS_error',lambda:True,ros_error,expected_exception=RCLError)
checks['live_context_error_failed_and_done_written']=r['completed']is False and r['fault']=='final_zero_publication_failed'and r['final_zero_publish_error'].startswith('RCLError')
def arbitrary_error():raise ValueError('real publication error')
r=fixture('arbitrary_error',lambda:True,arbitrary_error,expected_exception=ValueError)
checks['arbitrary_error_not_swallowed']=r['completed']is False and r['fault']=='final_zero_publication_failed'
run=OUT/'prepared_offline';contract=json.loads((run/'champ_contract.json').read_text());resolution=json.loads((run/'champ/loader_resolution.json').read_text())
checks['ELF_all_11_resolutions_no_missing']=len(resolution['libraries'])==11 and all(v['returncode']==0 and'not found'not in v['stdout']for v in resolution['libraries'].values())
exe=contract['selected_gait']['executable']['path'];dso=contract['selected_gait']['library']['path']
checks['selected_gait_DSO_not_old_staging']=('libquadruped_controller.so => '+dso+' ')in resolution['libraries'][exe]['stdout']
checks['GZ_uses_frozen_private_CM']=('libcontroller_manager.so => '+contract['controller_manager_library']['path']+' ')in resolution['libraries'][contract['gz_ros2_control_plugin']['path']]['stdout']
checks['all_8_message_DSOs_frozen_run_copies']=len(contract['loader']['copied_sha256'])==8 and all(contract['artifacts'].get(p)==h for p,h in contract['loader']['copied_sha256'].items())
checks['all_100_original_msg_and_index_refs_in_existing_scope_group']=len(contract['loader']['original_sha256'])==100 and all(contract['executor_source_sha256'].get(p)==h for p,h in contract['loader']['original_sha256'].items())
checks['message_copies_equal_original']=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in contract['loader']['copied_sha256'].items())
world=ET.parse(run/'world.sdf').getroot();plugin=world.find("world/model[@name='go2']/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
checks['Gazebo_plugin_filename_absolute_frozen']=plugin.get('filename')==contract['gz_ros2_control_plugin']['path']and Path(plugin.get('filename')).is_absolute()
checks['full_physical_scene_unchanged']=contract['physical_scene_identical_except_actuation_plugins']is True
checks['observer_binary_unmodified_from_V2']=hashlib.sha256((HERE/'native/build/libchamp_native_observer.so').read_bytes()).hexdigest()==hashlib.sha256((OUT/'legacy_v2/native/build/libchamp_native_observer.so').read_bytes()).hexdigest()
preserved=json.loads((OUT/'legacy_v2/preservation_receipt.json').read_text())
for p,h in preserved['sha256'].items():
    if '/runs/'in p:checks['actual_b5cf_original_unchanged_'+Path(p).name]=hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
checks['V2_21_original_report_preserved']=json.loads((OUT/'legacy_v2/tests/offline_report.json').read_text())['passed_count']==21
launchspec=importlib.util.spec_from_file_location('champ_loader_current_launch',HERE/'baseline.launch.py');lm=importlib.util.module_from_spec(launchspec);launchspec.loader.exec_module(lm)
from launch import LaunchContext
from launch.actions import OpaqueFunction
context=LaunchContext();context.launch_configurations['run_dir']=str(run)
actions=next(a for a in lm.generate_launch_description().entities if isinstance(a,OpaqueFunction)).execute(context)
checks['launch_offline_constructs']=len(actions)==8
report={'schema':1,'scope':'CHAMP-only loader and shutdown interface correction after actual startup failed b5cf; no controller/gait/physics/nav gain changes',
    'passed':all(checks.values()),'passed_count':sum(checks.values()),'total':len(checks),'checks':checks,'mock_cleanup_evidence':evidence,
    'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in[HERE/'prepare.py',HERE/'baseline.launch.py',HERE/'command_file_reader.py',Path(__file__)]},
    'native_so_sha256':hashlib.sha256((HERE/'native/build/libchamp_native_observer.so').read_bytes()).hexdigest(),
    'unchanged_policy_consumer_function_ast_sha256':m.consumer()[1],'original_actual_failed_run':'20261004_125401_navigation_champ_pid_v2_safety_r1_b5cf',
    'ROS_initialized':False,'simulator_started':False,'actual_CHAMP_motion_pass':False}
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));raise SystemExit(0 if report['passed']else 1)
