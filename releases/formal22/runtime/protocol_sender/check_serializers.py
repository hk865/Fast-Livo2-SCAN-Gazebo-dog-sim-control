#!/usr/bin/env python3
"""Execute actual serialization prefixes with retained P12 scalars; no ROS node."""
import pathlib,json,hashlib,subprocess,copy,time,re
H=pathlib.Path(__file__).resolve().parent;A=H.parents[2];E=H/'evidence';P=pathlib.Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261024T112001Z_closed_loop_R85_R137_P12_parking_transaction_round27_2e07');fixture=P/'path_admission_candidates/000002_trajectory_3.json';raw=fixture.read_bytes();assert hashlib.sha256(raw).hexdigest()=='3656c6ccdbd75e12846c408c901f5d343ca751b9d979398efce363fa4abe8e83';m=json.loads(raw)['metadata']
with(P/'r85_reference_attempts.jsonl').open()as f:
 for line_no,line in enumerate(f,1):
  assert line_no<=20
  attempt=json.loads(line)
  if attempt['attempt']['id']==3:break
 else:raise AssertionError('retained attempt3 absent')
old=pathlib.Path(json.loads((H/'SOURCE_COPY_MANIFEST.json').read_text())[0]['actual_source']).parents[1]/'src/scan_replan_fsm.cpp'
# Manifest order is not part of the contract: resolve the exact actual source.
old=pathlib.Path(next(x['actual_source']for x in json.loads((H/'SOURCE_COPY_MANIFEST.json').read_text())if x['file']=='src/scan_replan_fsm.cpp'))
new=H/'scan_ws/src/plan_manage/src/scan_replan_fsm.cpp'
pattern=r'r107_transport::Publication observation\(.*?observation.returned\(\);'
original_observations=re.findall(pattern,old.read_text(),re.S);actual_observations=re.findall(pattern,new.read_text(),re.S)
assert len(original_observations)==4 and original_observations==actual_observations
def method(path,name,newname,signature):
 s=path.read_text();a=s.index('  void SCANReplanFSM::'+name+'(');a=s.index('\n  {',a)+1;z=s.index('    std_msgs::msg::String',a);body=s[a:z].replace('return;','return {};')
 return 'std::string '+newname+signature+'\n'+body+'    return out.str();\n  }\n'
def j(x):return json.dumps(x,separators=(',',':'),ensure_ascii=True)
def literal(x):return j(j(x))
def vec(v):return '{'+','.join(map(repr,v))+'}'
a=m['attempt'];local=m['local_target'];st=m['start_state'];tr=m['trajectory']
header='''#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdint>
#include <unistd.h>
#include "plan_manage/r74_protocol_observation.h"
using namespace scan_planner;
namespace Eigen { struct Vector3d { double a=0,b=0,c=0; Vector3d()=default; Vector3d(double x,double y,double z):a(x),b(y),c(z){} double x()const{return a;} double y()const{return b;} double z()const{return c;} static Vector3d Zero(){return {};} }; }
struct Stamp {int32_t sec=0;uint32_t nanosec=0;};
namespace scan_planner_msgs {namespace msg {struct Bspline {int order=3,traj_id=0;Stamp start_time;struct Point {double x,y,z;};std::vector<Point> pos_pts;std::vector<double> knots,yaw_pts;double yaw_dt=0;};}}
std::string reference_curve_json(const std::string &s){return s;}
struct Global {double last_progress_time_=0,global_duration_=0;std::string global_traj_;}; struct Manager {Global global_data_;};
struct Harness {enum NAVI_MODE{REFERENCE_PATH=3};int navi_mode_=3;bool reference_ready_=true,external_reference_protocol_=true,go2_execution_frozen_=false;uint64_t reference_epoch_=0,localization_generation_=0,reference_revision_=0,candidate_attempt_id_=0,current_attempt_replan_sequence_=0;std::string reference_run_id_,reference_content_json_,attempt_origin_;Stamp reference_stamp_;double reference_original_duration_=0,attempt_progress_before_=0,attempt_progress_after_=0,local_target_time_=0;bool attempt_use_poly_init_=false,attempt_random_poly_=false;int global_endpoint_local_occupancy_=0,local_target_occupancy_=0;Eigen::Vector3d local_target_pt_,local_target_vel_,reference_body_goal_,end_pt_,odom_pos_,start_pt_,odom_vel_,start_vel_,start_acc_;Manager manager;Manager *planner_manager_=&manager;
'''
methods=method(old,'publishReferenceAttempt','old_attempt','(bool plan_success,const std::string &reason)')+method(new,'publishReferenceAttempt','new_attempt','(bool plan_success,const std::string &reason)')+method(old,'publishTrajectoryMetadata','old_meta','(const scan_planner_msgs::msg::Bspline &trajectory,bool emergency_stop)')+method(new,'publishTrajectoryMetadata','new_meta','(const scan_planner_msgs::msg::Bspline &trajectory,bool emergency_stop)')+method(new,'publishRouteDefinition','definition','()')
assign={'reference_epoch_':m['reference_epoch'],'localization_generation_':m['localization_generation'],'reference_revision_':m['committed_reference_revision'],'candidate_attempt_id_':a['id'],'current_attempt_replan_sequence_':a['local_replan_request_sequence'],'reference_run_id_':m['run_id'],'reference_original_duration_':m['reference_original_duration'],'attempt_origin_':a['origin'],'attempt_progress_before_':a['progress_before_target_selection'],'attempt_progress_after_':a['progress_after_target_selection'],'attempt_use_poly_init_':a['use_poly_init'],'attempt_random_poly_':a['random_poly'],'local_target_time_':local['time'],'global_endpoint_local_occupancy_':m['global_endpoint_local_grid']['occupancy'],'local_target_occupancy_':local['occupancy'],'go2_execution_frozen_':st['frozen_reference_boundary']}
main='};\nint main(){Harness h; scan_planner_msgs::msg::Bspline tr;\n'
main+=''.join('h.'+k+'='+j(v)+';\n'for k,v in assign.items());main+='h.reference_content_json_='+literal(m['reference_content'])+';\nh.manager.global_data_.global_traj_='+literal(m['global_state']['current_curve'])+';\n';main+=f'h.reference_stamp_={{{m["reference_stamp"][0]},{m["reference_stamp"][1]}}};\n';main+=f'h.manager.global_data_.last_progress_time_={m["global_state"]["progress"]!r};h.manager.global_data_.global_duration_={m["global_state"]["duration"]!r};\n'
for k,v in {'local_target_pt_':local['position'],'local_target_vel_':local['velocity'],'reference_body_goal_':m['body_goal'],'end_pt_':m['adjusted_body_goal'],'odom_pos_':st['position'],'start_pt_':st['position'],'odom_vel_':st['measured_velocity'],'start_vel_':st['used_velocity'],'start_acc_':st['used_acceleration']}.items():main+='h.'+k+'='+vec(v)+';\n'
main+=f'tr.order={tr["order"]};tr.traj_id={tr["traj_id"]};tr.start_time={{{tr["start_time"][0]},{tr["start_time"][1]}}};\n';main+='tr.pos_pts={'+','.join(vec(v)for v in tr['pos_pts'])+'};\n'
for k in['knots','yaw_pts']:main+='tr.'+k+'={'+','.join(map(repr,tr[k]))+'};\n'
main+='tr.yaw_dt='+repr(tr['yaw_dt'])+';\n'
for expr in ['h.old_attempt(true,"optimizer_return")','h.new_attempt(true,"optimizer_return")','h.old_meta(tr,false)','h.new_meta(tr,false)','h.definition()','h.new_attempt(false,"local_target_not_free_or_nonfinite")','h.new_meta(tr,true)']:
 main+='std::cout<<'+expr+'<<"\\n";\n'
main+='h.reference_stamp_.nanosec+=1; ++h.reference_revision_; std::cout<<h.definition()<<"\\n"; std::cout<<h.new_meta(tr,false)<<"\\n";return 0;}\n'
cpp=E/'serializer_harness.cpp';cpp.write_text(header+methods+main);exe=E/'serializer_harness';started=time.monotonic();subprocess.run(['/usr/bin/c++','-std=c++17','-O0','-I'+str(H/'scan_ws/src/plan_manage/include'),str(cpp),'-o',str(exe)],check=True,timeout=30);out=subprocess.check_output([str(exe)],timeout=5);rows=[json.loads(x)for x in out.splitlines()];oa,na,om,nm,definition,negative,emergency,update,updatem=rows
route=m['reference_stamp'][0]*10**9+m['reference_stamp'][1]
expecteda=copy.deepcopy(oa);del expecteda['reference_content'];expecteda.update(schema=3,route_id=route,route_version=m['committed_reference_revision']);assert expecteda==na
expectedm=copy.deepcopy(om);del expectedm['reference_content'];del expectedm['global_state']['current_curve'];trj=expectedm.pop('trajectory');expectedm.update(schema=3,route_id=route,route_version=m['committed_reference_revision'],trajectory_id=trj['traj_id'],trajectory_start_time=trj['start_time']);assert expectedm==nm
for k in ['reference_content','reference_original_duration','body_goal','run_id','reference_stamp','committed_reference_revision']:assert definition[k]==m[k]
assert not negative['attempt']['plan_success'] and negative['attempt']['result_reason']=='local_target_not_free_or_nonfinite';assert emergency['candidate_kind']=='emergency_stop' and not emergency['local_target']['valid'];assert update['route_id']==route+1 and update['route_version']==m['committed_reference_revision']+1 and updatem['route_version']==update['route_version']
# The retained real metadata is equivalent to the old actual serializer's fields.
expected_old=copy.deepcopy(m);expected_old['planner_process_id']=om['planner_process_id'];assert expected_old==om
checks={'schema':'scan_protocol_actual_serializer_software/v1','PASS':True,'original_R107_four_publication_observation_blocks_byte_identical':True,'no_ROS_or_DDS_node':True,'exact_actual_cpp_serialization_prefixes':True,'fixture':str(fixture),'fixture_sha256':hashlib.sha256(raw).hexdigest(),'attempt_line_read':line_no,'fixture_not_full_replay':True,'wall_s':time.monotonic()-started,'cases':['retained P12 trajectory3 exact old serializer fields','attempt only contracted deletion/schema/route ids','metadata only contracted deletions/schema/route/trajectory ids','one definition full reference original duration/body goal','negative attempt remains negative','emergency small fields remain','stamp/revision replacement IDs'],'bytes':{'old_attempt_utf8':len(out.splitlines()[0]),'new_attempt_utf8':len(out.splitlines()[1]),'old_metadata_utf8':len(out.splitlines()[2]),'new_metadata_utf8':len(out.splitlines()[3]),'definition_utf8':len(out.splitlines()[4])},'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [old,new]},'DDS_QoS_or_late_subscriber_runtime_proven':False}
(E/'SERIALIZER_CHECKS.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks))
