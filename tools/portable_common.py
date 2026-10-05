"""Portable locators and fail-closed local build gate; no historical PASS reuse."""
from pathlib import Path
import hashlib,json,os,xml.etree.ElementTree as ET
REPO=Path(__file__).resolve().parents[1]
LOCAL=REPO/'.local'
MODEL_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
NAMES={'v12':'lidar_sampling_v12','v18':'combined_compute_v18','v17':'ingress_pipeline_v17'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def model_path():return Path(os.environ.get('TEACHER_MODEL_CHECKPOINT',str(REPO/'models/model_1000.pt'))).expanduser().resolve()
def cpu_python():return Path(os.environ.get('TEACHER_CPU_PYTHON',os.sys.executable)).expanduser().resolve()
def asset_path(uri):
    if '/unitree_go2_description/meshes/'in uri:
        tail=uri.split('/unitree_go2_description/meshes/',1)[1]
        p=REPO/'go2_sim_control/src/unitree_go2_ros2/unitree_go2_description/meshes'/tail
    elif uri in ('model://realsense2_description/meshes/d455.stl','package://realsense2_description/meshes/d455.stl'):
        p=REPO/'tools/assets/realsense2_description/meshes/d455.stl'
    elif uri.startswith('file://'):p=Path(uri[7:])
    else:raise ValueError('Unresolved portable asset URI '+uri)
    p=p.resolve()
    if not p.is_file()or not p.is_relative_to(REPO):raise ValueError('Asset unavailable or outside repository '+str(p))
    return p

def resolve_tree_assets(root):
    evidence=[]
    for element in root.iter():
        if element.tag=='uri'and (element.text or '').strip():
            original=element.text.strip();p=asset_path(original);element.text='file://'+str(p);evidence.append({'source_uri':original,'resolved':str(p),'sha256':sha(p)})
        elif element.tag=='mesh'and element.get('filename'):
            original=element.get('filename');p=asset_path(original);element.set('filename','file://'+str(p));evidence.append({'source_uri':original,'resolved':str(p),'sha256':sha(p)})
    return evidence

def gate_path(here):
    name=Path(here).name
    variant=next((k for k,v in NAMES.items()if v==name),None)
    if variant not in ('v18','v17'):raise RuntimeError('Only V18/V17 portable experimental runtime is supported')
    return LOCAL/('LOCAL_PREFLIGHT_'+variant.upper()+'.json')
def evidence_files(here):
    p=gate_path(here)
    if not p.is_file():return [p]
    d=json.loads(p.read_text());return [p]+[REPO/x for x in d.get('repository_bindings',{})]+[Path(x)for x in d.get('local_artifact_bindings',{})]
def verify_local_preflight(here,profile):
    here=Path(here).resolve();p=gate_path(here)
    if not p.is_file():raise RuntimeError('Run tools/local_preflight.py after tools/build_local.py; historical preflight does not authorize this clone')
    d=json.loads(p.read_text())
    if d.get('schema')!='portable_local_finite_preflight/v1' or d.get('status')!='PASS_LOCAL_BUILD_FINITE_CHECKS_EXPERIMENTAL':raise RuntimeError('Local finite build gate did not pass')
    if d.get('repo')!=str(REPO)or d.get('candidate_root')!=str(here)or d.get('actual_simulation_verified')is not False or d.get('historic_pass_inherited')is not False:raise RuntimeError('Local experimental preflight identity/scope differs')
    for rel,digest in d.get('repository_bindings',{}).items():
        if sha(REPO/rel)!=digest:raise RuntimeError('Portable reviewed source changed '+rel)
    for path,digest in d.get('local_artifact_bindings',{}).items():
        if sha(path)!=digest:raise RuntimeError('Local build/numeric evidence changed '+path)
    if not d.get('finite_numeric_all_byte_identical'):raise RuntimeError('Required local finite numeric comparisons did not pass')
    required=[here/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',here/'slam_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
    if any(str(x)not in d['local_artifact_bindings']for x in required):raise RuntimeError('Private loaded binary is not bound')
    if profile.get('pose_cloud_timeout_s')!=.3 or profile.get('navigation_ground_truth_used')is not False:raise RuntimeError('Original freshness and actual-SLAM-only control scope required')
    if d['variant']=='v18':
        lio=profile.get('lio_jacobian_parallelism',{});vio=profile.get('vio_patch_parallel',{})
        if lio.get('threads')!=4 or lio.get('parallel_min_rows')!=256 or vio.get('threads')!=1 or vio.get('serial_below_patch_count')!=64:raise RuntimeError('Local V18 profile requires LIO4/VIO1')
    return d
