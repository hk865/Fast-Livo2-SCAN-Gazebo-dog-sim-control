"""Only newly reviewed V19 source/build/test evidence can permit an experiment."""
from pathlib import Path
import hashlib
import json

if not __debug__:
    raise RuntimeError('V19 preflight forbids optimized Python')

REQUIRED=frozenset(('independent_build','queue_lifecycle','production_packet_semantics',
    'finite_math','fp_team_loader','control_reference_revision_reviewed',
    'lifecycle_reader_negative_tests','prospective_protocol'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_files(here):
    path=Path(here)/'PIPELINE_V19_PREFLIGHT.json'
    if not path.is_file():raise RuntimeError('V19 has no independently completed preflight yet')
    d=json.loads(path.read_text())
    return [path]+[Path(x)for x in d.get('verified_inputs_sha256',{})]


def verify_preflight(here,profile):
    here=Path(here).resolve()
    path=here/'PIPELINE_V19_PREFLIGHT.json'
    if not path.is_file():raise RuntimeError('V19 fresh build/queue/production/numeric checks are required')
    d=json.loads(path.read_text())
    if (d.get('schema')!='pipeline_v19_preflight/v1' or d.get('status')!='PASS_LIMITED_NEW_EXPERIMENT'
        or d.get('candidate_root')!=str(here)or d.get('allowed')is not True
        or d.get('actual_navigation_verified')is not False or d.get('historical_pass_inherited')is not False):
        raise RuntimeError('Wrong V19 identity, finite scope or inherited acceptance')
    if set(d.get('checks',{}))!=REQUIRED or any(v is not True for v in d['checks'].values()):
        raise RuntimeError('V19 required independent checks have not all passed')
    p=profile.get('pipeline',{})
    if (p.get('schema')!='pipeline_v19_contract/v1'or p.get('mode')not in ('serial','rx_decode','staged')
        or type(p.get('image_copy_opt'))is not int or p['image_copy_opt']not in(0,1)
        or p.get('max_items')!=512 or p.get('reserved_bytes')!=67108864):
        raise RuntimeError('Explicit bounded V19 pipeline mode is required')
    combination={'mode':p['mode'],'image_copy_opt':p['image_copy_opt']}
    if combination not in d.get('verified_mode_combinations',[]):
        raise RuntimeError('This mode/copy combination was not independently tested')
    if profile.get('pose_cloud_timeout_s')!=.3 or profile.get('navigation_ground_truth_used')is not False:
        raise RuntimeError('V19 source freshness and actual-SLAM-only navigation remain required')
    if profile.get('lio_jacobian_parallelism',{}).get('threads')!=4 or profile.get('vio_patch_parallel',{}).get('threads')!=1:
        raise RuntimeError('V19 retains V18 LIO4/VIO1 original ordered arithmetic')
    required=list(here.glob('*.py'))+[here/'PIPELINE_V19_CONTRACT.json']
    required +=[x for x in(here/'slam_ws/src').rglob('*')if x.is_file()and
                (x.suffix in('.cpp','.h','.hpp')or x.name in('CMakeLists.txt','package.xml'))]
    required +=[here/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',
                 here/'slam_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
    required +=[here/'slam_ws/build'/package/name for package in('fast_livo2_core','fast_livo2_ros')
                for name in('compile_commands.json','CMakeCache.txt')]
    core=here.parents[1]/'test_results/pipeline_v19_20261006/core/CORE_PREFLIGHT.json'
    required +=[core]
    bindings=d.get('verified_inputs_sha256',{})
    if not bindings or any(str(x.resolve())not in bindings for x in required):
        raise RuntimeError('V19 mandatory source/build/test bindings are incomplete')
    for name,digest in bindings.items():
        if not Path(name).is_absolute()or sha(name)!=digest:
            raise RuntimeError('V19 reviewed source or evidence changed: '+name)
    return d
