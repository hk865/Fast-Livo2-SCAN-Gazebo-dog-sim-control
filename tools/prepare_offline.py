#!/usr/bin/env python3
"""No ROS/torch: generate bounded source/assets preview, never runtime permission."""
from pathlib import Path
if not __debug__:
 raise RuntimeError("Optimized Python (-O/PYTHONOPTIMIZE) is forbidden for portable validation")
import argparse,hashlib,json,subprocess,sys,xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
from portable_common import REPO,MODEL_SHA,resolve_tree_assets,sha,NAMES
TEACHER=REPO/'multifloor_demo/teacher_mode'
def main():
 p=argparse.ArgumentParser();p.add_argument('--variant',choices=['v18','v17'],required=True);p.add_argument('--profile',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--model',type=Path);a=p.parse_args()
 cfgpath=TEACHER/'navigation'/NAMES[a.variant]/'profiles'/(a.profile if a.profile.endswith('.json')else a.profile+'.json');cfg=json.loads(cfgpath.read_text())
 out=a.output.expanduser().resolve()
 if out.exists():raise RuntimeError('Refuse existing offline preview directory')
 out.mkdir(parents=True,mode=0o700)
 subprocess.run([sys.executable,'-B',str(TEACHER/'simulation/prepare.py'),'--output',str(out),'--terrain',cfg['expected_asset']['terrain'],'--sensors','--camera-rate','10'],check=True)
 world=ET.parse(out/'world.sdf');robot=world.getroot().find("world/model[@name='go2']");robot.find('pose').text=' '.join(map(str,[*cfg['spawn'][:3],0,0,cfg['spawn'][3]]))
 sys.path.insert(0,str(cfgpath.parent.parent));from sampling import override_sensors
 override_sensors(out,cfg)
 world=ET.parse(out/'world.sdf');uris=resolve_tree_assets(world.getroot());ET.indent(world);world.write(out/'world.sdf',encoding='unicode')
 robot=world.getroot().find("world/model[@name='go2']");plugins=robot.findall('plugin');act=[x for x in plugins if x.get('name')=='teacher_sim::TeacherActuator'];assert len(act)==1
 assert cfg['pose_cloud_timeout_s']==.3 and cfg['navigation_ground_truth_used']is False
 assert cfg['sensor_sampling']['vertical_lines']==64 and cfg['sensor_sampling']['horizontal_samples']==480
 # The complete profile owns the32 region route; smoke/prefix uses same route but a limited horizon.
 goals=cfg.get('route_world_points')
 policy=json.loads((TEACHER/'policy/contract.json').read_text())
 expected=[f'{leg}_{part}_joint'for leg in('rf','lf','rh','lh')for part in('hip','upper_leg','lower_leg')]
 assert policy['gazebo_joint_names']==expected
 assert len(goals)==32
 assert float(world.getroot().findtext('world/physics/max_step_size'))==.005
 model={'status':'EXTERNAL_NOT_CHECKED','expected_sha256':MODEL_SHA}
 if a.model:
  model={'status':'HASH_VERIFIED_ONLY','path':str(a.model.resolve()),'sha256':sha(a.model)};assert model['sha256']==MODEL_SHA
 receipt={'schema':'portable_offline_source_assets/v1','status':'PASS_SOURCE_ASSETS_ONLY','runtime_allowed':False,'requires_local_source_build_and_finite_preflight':True,'ROS_started':False,'simulation_started':False,'torch_imported':False,'variant':a.variant,'profile':str(cfgpath.relative_to(REPO)),'profile_sha256':sha(cfgpath),'output':str(out),'asset_uris':uris,'model':model,'one_joint_writer':True,'actual_navigation_uses_truth':False,'route_goal_field':goals,'nominal_policy_hz':50,'nominal_physics_hz':200,'world_sha256':sha(out/'world.sdf'),'limits':'Generated preview only, not motion/Sim2Sim/navigation pass. Historical result is not inherited.'}
 (out/'OFFLINE_PREPARATION.json').write_text(json.dumps(receipt,indent=2)+'\n');print('PASS_SOURCE_ASSETS_ONLY; runtime_allowed=false')
if __name__=='__main__':main()
