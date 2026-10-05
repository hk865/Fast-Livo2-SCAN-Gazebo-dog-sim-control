#!/usr/bin/env python3
"""Read-only scene/source/host audit. Writes only fresh receipts beside itself."""
from __future__ import annotations
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import urllib.request
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent
T = OUT.parents[2]
DEMO = T.parent
REPO = DEMO.parent
SLAM_RUN = T/'runs/20261005_014633_SLAM_fixed_route_v3_flat6m_mapped10_readtime_v5_r3_8bf5'
REGISTRATION = T/'test_results/ramp_registration_actual_20261004/stand2071/registration/registration.json'
CAMERA_MISSION = DEMO/'camera_mode/runs/20261002_190238_5466b5/mission.json'


def digest(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()


def read(p): return json.loads(Path(p).read_text())
def write_new(name,obj):
    with (OUT/name).open('x') as f:
        json.dump(obj,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')


def cmd(args):
    try:
        r=subprocess.run(args,text=True,capture_output=True,timeout=10)
        return {'command':args,'returncode':r.returncode,'stdout':r.stdout.strip(),'stderr':r.stderr.strip()}
    except Exception as e: return {'command':args,'error':repr(e)}


def cpu_ticks():
    a=[int(x)for x in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
    return sum(a),a[3]+a[4]


def host():
    begin=dt.datetime.now(dt.timezone.utc).isoformat()
    a=cpu_ticks();time.sleep(.5);b=cpu_ticks()
    mem={}
    for line in Path('/proc/meminfo').read_text().splitlines():
        k,v=line.split(':',1);mem[k]=int(v.split()[0])
    found=[]
    needles=('teacher_mode/scripts/serve.py','teacher_mode/navigation/truth_tuning_design/dashboard.py',
             'teacher_mode/navigation/truth_tuning_slam/slam_transfer_dashboard.py',
             'gz sim','gz-sim','fastlivo_mapping','scan_planner_node','RL_for_unitree','isaaclab',
             'teacher_mode/policy/worker.py','truth_tuning_slam/worker.py','curvature_tuning/plant_worker.py')
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name)==os.getpid(): continue
        try:
            argv=p.joinpath('cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace').strip()
            if not argv or not any(x in argv for x in needles):continue
            status=p.joinpath('status').read_text().splitlines()
            selected={s.split(':',1)[0]:s.split(':',1)[1].strip() for s in status if s.startswith(('State:','VmRSS:','Threads:'))}
            env={}
            for e in p.joinpath('environ').read_bytes().split(b'\x00'):
                k,sep,v=e.partition(b'=')
                if k.decode(errors='ignore') in ('ROS_DOMAIN_ID','GZ_PARTITION','OMP_NUM_THREADS','FASTRTPS_DEFAULT_PROFILES_FILE','RMW_IMPLEMENTATION'):
                    env[k.decode()]=v.decode(errors='replace')
            found.append({'pid':int(p.name),'argv':argv,**selected,'selected_environment':env})
        except (OSError,ValueError):pass
    viewers=[]
    for port in (8768,8769,8770):
        t=dt.datetime.now(dt.timezone.utc).isoformat()
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/',timeout=3) as r:
                viewers.append({'port':port,'sample_utc':t,'http_status':r.status,'content_type':r.headers.get('Content-Type'),
                                'initial_body_bytes_read':len(r.read(128)),'read_only_GET':True})
        except Exception as e:viewers.append({'port':port,'sample_utc':t,'error':repr(e),'read_only_GET':True})
    disk=shutil.disk_usage(REPO)
    return {'schema':'read_only_host_snapshot/v1','started_utc':begin,'finished_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
        'timezone':'Asia/Shanghai','signals_sent':False,'processes_started':False,
        'logical_cpus':os.cpu_count(),'load_1_5_15':list(os.getloadavg()),
        'cpu_sample_interval_s':.5,'cpu_busy_percent':100*(1-(b[1]-a[1])/(b[0]-a[0])),
        'memory_KiB':{k:mem[k]for k in ('MemTotal','MemAvailable','SwapTotal','SwapFree')},
        'disk_bytes':{'total':disk.total,'free':disk.free},'matching_processes':found,
        'nvidia_summary':cmd(['nvidia-smi','--query-gpu=name,memory.used,memory.total,utilization.gpu,utilization.memory','--format=csv,noheader,nounits']),
        'nvidia_compute_processes':cmd(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader,nounits']),
        'port_listeners':cmd(['ss','-ltnp','sport = :8768 or sport = :8769 or sport = :8770']),
        'viewer_GET':viewers,'limits':'Point-in-time observation only; no assertion that external training will remain idle.'}


def main():
    for n in ('host_audit.json','scene_audit.json','input_hashes.json'):
        if (OUT/n).exists():raise RuntimeError('Refusing to overwrite '+n)
    files=[T/'navigation/scoped_profile.py',T/'navigation/slam.launch.py',T/'navigation/stack.launch.py',
           T/'navigation/controller.py',DEMO/'navigation/trajectory_contract.py',
           T/'navigation/truth_tuning_slam/run.py',T/'navigation/truth_tuning_slam/adapter.py',
           T/'navigation/ramp_registration/geometry.py',T/'navigation/ramp_registration/analyze.py',
           T/'simulation/prepare.py',T/'simulation/build/libteacher_actuator.so',T/'policy/contract.json',
           T/'tests/terrain_targets/lower12.json',T/'tests/terrain_targets/upper23.json',
           T/'current_status.json',DEMO/'slam/heading_alignment.py',DEMO/'scripts/mission_server.py',
           DEMO/'mission/state_machine.py',DEMO/'navigation/controller.py',DEMO/'navigation/control_core.py',
           DEMO/'camera_mode/simulation/scenario.json',DEMO/'simulation/generated/three_floors.sdf',
           SLAM_RUN/'sensor_contract.json',SLAM_RUN/'asset_manifest.json',SLAM_RUN/'navigation_fastlivo.yaml',
           SLAM_RUN/'SLAM_fixed_route/adapter_IMU_inputs.jsonl',REGISTRATION,CAMERA_MISSION]
    files += sorted(T.glob('runs/20261005_*readtime_v5_*/summary_slam_transfer_independent.json'))
    sources={str(p):digest(p)for p in files}
    original_scene=read(DEMO/'camera_mode/simulation/scenario.json')
    world=ET.parse(DEMO/'simulation/generated/three_floors.sdf').getroot().find('world')
    geom={}
    for m in world.findall('model'):
        if m.get('name')not in ('floor_1','floor_2','floor_3','ramp_12','ramp_23'):continue
        c=m.find('link/collision')
        geom[m.get('name')]={'model_pose_xyz_rpy':[float(v)for v in m.findtext('pose').split()],
            'collision_pose_xyz_rpy':[float(v)for v in c.findtext('pose','0 0 0 0 0 0').split()],
            'box_size_m':[float(v)for v in c.findtext('geometry/box/size').split()]}
    imus=[json.loads(l)for l in (SLAM_RUN/'SLAM_fixed_route/adapter_IMU_inputs.jsonl').open()if l.strip()]
    reg=read(REGISTRATION);h=read(CAMERA_MISSION)['heading_alignment']
    observed={
        'schema':'read_only_teacher_closed_loop_scene_audit/v1',
        'audit_utc':dt.datetime.now(dt.timezone.utc).isoformat(),
        'scope':'Simulation preparation only; no runtime launch or acceptance promotion',
        'new_closed_loop_multifloor_actual_verified':False,
        'existing_scope':{'file':str(T/'navigation/scoped_profile.py'),'limited_to_flat':True,
             'spawn':[6,-.7,.4,0],'historical_motion_basis_required':30,
             'new_user_authorized_multifloor_requires_independent_scope':True},
        'actual_SLAM_fixed_route_evidence':[{'file':str(p),'sha256':sources[str(p)],'status':read(p)['status'],
                                           'levels':read(p)['levels']}for p in sorted(T.glob('runs/20261005_*readtime_v5_*/summary_slam_transfer_independent.json'))],
        'actual_IMU_orientation':{'run':str(SLAM_RUN),'source_file':str(SLAM_RUN/'SLAM_fixed_route/adapter_IMU_inputs.jsonl'),
             'rows':len(imus),'orientation_available_rows':sum(r.get('orientation_available')is True for r in imus),
             'first_actual_row':imus[0],'reference':read(SLAM_RUN/'sensor_contract.json')['imu']['orientation_reference']},
        'existing_actual_camera_heading_example':{'run':str(CAMERA_MISSION.parent),
             **{k:v for k,v in h.items()if k!='attitude_pairs'},
             'reusing_this_old_numeric_transform_for_new_Teacher_run_is_forbidden':True},
        'known_scene_geometry_prior':geom,
        'existing_camera_route_prior':{'source':str(DEMO/'camera_mode/simulation/scenario.json'),
             'coordinates':original_scene['coordinates'],'waypoint_reference':original_scene['waypoint_reference'],
             'old_camera_scene_anchor':original_scene['slam_origin_in_world'],
             'navigation_f1_f3':original_scene['navigation_f1_f3'],
             'not_a_Teacher_multifloor_pass':True,'do_not_reuse_camera_body_height':True},
        'actual_pointcloud_registration':{'file':str(REGISTRATION),
             **{k:reg.get(k)for k in ('schema','status','frame_id','ground_truth_used','full_route_eligible','full_route',
                  'rejection_reasons','up_camera_init','body_anchor','input_points')},
             'support_plane_count':len(reg.get('support_planes',[])),'ramp_candidate_count':len(reg.get('ramp_candidates',[])),
             'registration_control_enabled':False,'navigation_request_emitted':False},
        'fixed_privileged_height_providers':{name:read(T/f'tests/terrain_targets/{name}.json')for name in ('lower12','upper23')},
        'proposed_stage_order':['actual SLAM/SCAN flat route and map-prior registration check',
               'ramp12 complete entry x1.2,y2 -> exit x14.8,y2, relative support +1.2m',
               'floor2 connector x16,y2 -> x16,y7, then ramp23 entry x14.8,y7',
               'ramp23 complete x14.8,y7 -> x1.2,y7, relative support +1.2m'],
        'navigation_source_boundary':{'feedback':'actual SLAM body_odom and original IMU gyro/orientation',
             'safety':'actual registered cloud and checked SCAN, 300ms freshness',
             'registration':'new-run frozen sensor axis calibration plus explicitly declared known-map initial placement prior, or actual cloud landmark registration',
             'arrival':'strictly increasing original SLAM header stamps; no native truth correction',
             'native_truth':'offline only for clearance/contact/route error/q/tau; no navigation state injection',
             'actor':'Frozen Teacher native privileged state/187-height observations remain explicitly privileged; not sensory replacement validation'},
        'no_open_loop_success_precondition_for_new_authorized_closed_loop_scope':True,
        'unknowns':['No actual full multifloor SLAM/SCAN navigation result with the new controller yet',
                    'No current automatically confirmed full ramp/unique support-layer map',
                    'Known initial placement is a prior, with settling/registration uncertainty',
                    'SCAN body-height and layer-overlap semantics require the new scope to retain exact conventions',
                    'Active endpoint hold must yield to stale/tilt/obstacle safety before emitting velocity'],
        'input_source_hashes':sources}
    write_new('host_audit.json',host())
    observed['host_audit_file']=str(OUT/'host_audit.json');observed['host_audit_sha256']=digest(OUT/'host_audit.json')
    after={str(p):digest(p)for p in files}
    observed['original_inputs_unchanged_during_audit']=sources==after
    if sources!=after: observed['changed_during_audit']=[k for k in sources if sources[k]!=after[k]]
    write_new('scene_audit.json',observed)
    write_new('input_hashes.json',{'schema':'read_only_audit_provenance/v1','sources_before':sources,'sources_after':after,
             'original_inputs_unchanged':sources==after,'auditor_sha256':digest(__file__),
             'outputs':{n:digest(OUT/n)for n in ('host_audit.json','scene_audit.json')}})
    print(json.dumps({'scene_audit':str(OUT/'scene_audit.json'),'sha256':digest(OUT/'scene_audit.json'),
                      'host_audit_sha256':digest(OUT/'host_audit.json'),'inputs_unchanged':sources==after}))


if __name__=='__main__':main()
