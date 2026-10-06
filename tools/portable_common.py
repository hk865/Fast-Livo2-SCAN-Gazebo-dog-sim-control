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
def required_repository_files(here):
    teacher=REPO/'multifloor_demo/teacher_mode'
    paths=[REPO/'tools/local_preflight.py',REPO/'tools/portable_common.py',REPO/'tools/portable_transport.py',teacher/'policy/worker.py',teacher/'policy/observation.py',teacher/'policy/contract.json',teacher/'simulation/prepare.py',teacher/'simulation/teacher_actuator.cpp']
    paths +=[teacher/'test_results/parallel_pipeline_20261005'/x for x in ['lio_v15/finite_lio_vio_fixture.cpp','lio_v15/observe_gomp.cpp','vio_v16/vio_patch_fixture.cpp','vio_v16/run_numeric_fixtures.py']]
    paths+=list(here.glob('*.py'))+list((here/'profiles').glob('*.json'))
    paths +=[x for x in(here/'slam_ws/src').rglob('*')if x.is_file()and(x.suffix in('.cpp','.h','.hpp')or x.name in('CMakeLists.txt','package.xml'))]
    if len(paths)<25:raise RuntimeError('Incomplete portable candidate source export')
    return sorted(set(paths))
def verify_local_preflight(here,profile):
    if not __debug__:raise RuntimeError('Optimized Python validation forbidden')
    here=Path(here).resolve();p=gate_path(here)
    if not p.is_file():raise RuntimeError('Run tools/local_preflight.py after tools/build_local.py; historical preflight does not authorize this clone')
    d=json.loads(p.read_text());variant=next(k for k,n in NAMES.items()if n==here.name)
    if d.get('schema')!='portable_local_finite_preflight/v1' or d.get('status')!='PASS_LOCAL_BUILD_FINITE_CHECKS_EXPERIMENTAL':raise RuntimeError('Local finite build gate did not pass')
    if d.get('repo')!=str(REPO)or d.get('candidate_root')!=str(here)or d.get('variant')!=variant or d.get('actual_simulation_verified')is not False or d.get('historic_pass_inherited')is not False:raise RuntimeError('Local experimental preflight identity/scope differs')
    bindings=d.get('repository_bindings');artifacts=d.get('local_artifact_bindings')
    if not isinstance(bindings,dict)or not isinstance(artifacts,dict)or not bindings or not artifacts:raise RuntimeError('Missing local source/artifact bindings')
    mandatory={str(x.relative_to(REPO))for x in required_repository_files(here)}
    if not mandatory<=bindings.keys():raise RuntimeError('Missing mandatory reviewed candidate/producer source bindings')
    for rel,digest in bindings.items():
        path=(REPO/rel).resolve()
        if not path.is_relative_to(REPO)or sha(path)!=digest:raise RuntimeError('Portable reviewed source changed '+rel)
    for path,digest in artifacts.items():
        actual=Path(path).resolve()
        if not actual.is_relative_to(REPO)or sha(actual)!=digest:raise RuntimeError('Local build/numeric evidence changed '+path)
    numeric=Path(d.get('finite_receipt','')).resolve();fp=Path(d.get('FP_team_loader_receipt','')).resolve();buildpath=LOCAL/'SOURCE_BUILD_RECEIPT.json'
    required=[here/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',here/'slam_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',REPO/'multifloor_demo/teacher_mode/simulation/build/libteacher_actuator.so',numeric,fp,buildpath]
    if any(str(x)not in artifacts for x in required):raise RuntimeError('Private binaries or mandatory fresh finite/FP/build evidence are not bound')
    build=json.loads(buildpath.read_text());finite=json.loads(numeric.read_text());observed=json.loads(fp.read_text())
    if build.get('status')!='PASS_SOURCE_BUILD_ONLY'or not build.get('source_copy_proofs')or not build.get('ambient_project_search_variables_removed'):raise RuntimeError('Source build provenance incomplete')
    for v in ('v12',variant):
        sources=build['variants'][v].get('source_sha256',{})
        if not sources or any(sha(REPO/path)!=digest for path,digest in sources.items()):raise RuntimeError('Compiled source no longer matches fresh build inputs')
    if finite.get('schema')!='portable_fresh_finite_checks/v1'or finite.get('all_byte_identical')is not True or finite.get('historical_results_consumed')is not False or finite.get('process_runs',0)<100:raise RuntimeError('Fresh finite numeric receipt did not pass')
    if not {'v12',variant}<=finite.get('variants',{}).keys():raise RuntimeError('Finite baseline/candidate identity absent')
    if not all(len(finite['variants'][v]['LIO']['runs'])==9 and len(finite['variants'][v]['VIO41']['runs'])==41 for v in ('v12',variant)):raise RuntimeError('Finite case coverage incomplete')
    for v in ('v12',variant):
        data=finite['variants'][v]
        for label,digest in data['LIO']['runs'].items():
            file=numeric.parent/(v+'_lio')/(label+'.bin')
            if artifacts.get(str(file))!=digest:raise RuntimeError('Finite state output binding missing')
        for label,digest in data['VIO41']['runs'].items():
            file=numeric.parent/(v+'_patch')/label/'output.bin'
            if artifacts.get(str(file))!=digest:raise RuntimeError('Finite patch output binding missing')
    state=[c['full_729_StateEstimation_and_VIO9_cases']for c in finite['comparisons']if c.get('variant')==variant and 'full_729_StateEstimation_and_VIO9_cases'in c]
    patch=[c['VIO41_boundary_cases']for c in finite['comparisons']if c.get('variant')==variant and 'VIO41_boundary_cases'in c]
    if len(state)!=1 or len(state[0])!=3 or len(patch)!=1 or len(patch[0])!=41 or not all(all(value is True for key,value in row.items()if key!='scene'and key!='case')for row in state[0]+patch[0]):raise RuntimeError('Finite comparison rows do not prove required byte equivalence')
    if not d.get('finite_numeric_all_byte_identical')or observed.get('schema')!='portable_test_only_FP_team_loader/v1'or observed.get('all_observed_FP_valid')is not True or observed.get('all_observed_outputs_byte_identical')is not True or not {'v12',variant}<=observed.get('variants',{}).keys():raise RuntimeError('Fresh finite FP/team/loader evidence incomplete')
    for v in ('v12',variant):
        witness=observed['variants'][v]
        if witness.get('library_sha256')!=build['variants'][v]['artifacts_sha256'][str(REPO/'multifloor_demo/teacher_mode/navigation'/NAMES[v]/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so')]or not witness.get('loaded_private_library_verified')or witness.get('main_FP_processes',0)<2:raise RuntimeError('Actual private library and main FP observation missing')
    candidates=[x for x in(here/'profiles').glob('*.json')if json.loads(x.read_text())==profile]
    if len(candidates)!=1:raise RuntimeError('Profile must exactly equal one frozen candidate profile')
    if profile.get('pose_cloud_timeout_s')!=.3 or profile.get('navigation_ground_truth_used')is not False:raise RuntimeError('Original freshness and actual-SLAM-only control scope required')
    if variant=='v18':
        lio=profile.get('lio_jacobian_parallelism',{});vio=profile.get('vio_patch_parallel',{})
        if lio.get('threads')!=4 or lio.get('parallel_min_rows')!=256 or vio.get('threads')!=1 or vio.get('serial_below_patch_count')!=64:raise RuntimeError('Local V18 profile requires LIO4/VIO1')
        if observed['variants'][variant].get('LIO_actual_team')!=4 or observed['variants'][variant].get('VIO_actual_team')!=1:raise RuntimeError('Private V18 actual parallel team differs')
    if variant=='v17':raise RuntimeError('Portable V17 queue semantic replay is not implemented; preview/build/numeric only')
    return d
