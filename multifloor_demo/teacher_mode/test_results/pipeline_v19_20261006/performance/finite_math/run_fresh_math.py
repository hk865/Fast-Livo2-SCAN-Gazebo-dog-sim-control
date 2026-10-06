#!/usr/bin/env python3
"""Fresh finite V18/V19 numerical regression + observed FP/team/private loader.

No ROS graph, Gazebo, controller, Teacher, statistics benchmark or old PASS reuse.
"""
import concurrent.futures
import hashlib
import json
import os
import shlex
import struct
import subprocess
from pathlib import Path

if not __debug__:
    raise RuntimeError('Finite validation must not run with -O/PYTHONOPTIMIZE')
HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[3]
PROJECT = TEACHER.parents[1]
VARIANTS = {'baseline_V18': TEACHER/'navigation/combined_compute_v18/slam_ws',
            'candidate_V19': TEACHER/'navigation/pipeline_v19/slam_ws'}
V19_SHA = 'ec5ccfab27028108841d720447e35e902b148b89ffc56cc23f108f25ee8c0e2a'
AFFINITY = '0,2,4,6'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def require(value, message):
    if not value: raise RuntimeError(message)


def compile_fixture(variant, source, name):
    ws = VARIANTS[variant]; target = HERE/variant/name; target.mkdir(parents=True, exist_ok=False)
    lib = ws/'install/fast_livo2_core/lib/libfast_livo2_core.so'
    if variant == 'candidate_V19': require(sha(lib) == V19_SHA, 'V19 private library changed')
    flags = {}
    flags_file = ws/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make'
    for line in flags_file.read_text().splitlines():
        if line.startswith('CXX_'):
            k, v = line.split('=', 1); flags[k.strip()] = shlex.split(v)
    vikit = PROJECT/'slam5_navigation/ros2_ws/install/vikit_common/lib/libvikit_common.so'
    cmd = ['/usr/bin/c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
           str(source), str(lib), str(vikit), '/usr/lib/x86_64-linux-gnu/libpcl_common.so',
           '/usr/lib/x86_64-linux-gnu/libopencv_core.so', '/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so',
           '/opt/ros/jazzy/lib/librclcpp.so', '-Wl,-rpath,'+str(lib.parent),
           '-Wl,-rpath-link,/opt/ros/jazzy/lib', '-o', str(target/'fixture')]
    env = os.environ.copy()
    for key in ('LD_PRELOAD', 'FASTLIVO_DIAGNOSTIC_DIR', 'FASTLIVO_PIPELINE_MODE', 'FASTLIVO_IMAGE_COPY_OPT'):
        env.pop(key, None)
    env['LD_LIBRARY_PATH'] = ':'.join([str(lib.parent), '/opt/ros/jazzy/lib', '/opt/ros/jazzy/lib/x86_64-linux-gnu',
        '/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib', str(vikit.parent),
        str(PROJECT/'slam5_navigation/ros2_ws/install/vikit_ros/lib')])
    env.update(OMP_NUM_THREADS='1', OMP_DYNAMIC='FALSE', OPENBLAS_NUM_THREADS='1',
               FASTLIVO_BOUNDARY_TIMING='0', FASTLIVO_LIO_JACOBIAN_THREADS='4', FASTLIVO_VIO_PATCH_THREADS='1',
               MAKEFLAGS='-j2 -l2', CMAKE_BUILD_PARALLEL_LEVEL='2')
    r = subprocess.run(cmd, env=env, text=True, capture_output=True)
    (target/'compile.log').write_text(r.stdout+r.stderr)
    require(r.returncode == 0, 'Fixture compile failed '+variant+'/'+name+': '+r.stderr[-2000:])
    return {'variant': variant, 'name': name, 'target': str(target), 'library': str(lib),
            'library_sha256': sha(lib), 'flags_sha256': sha(flags_file), 'source_sha256': sha(source),
            'compile_argv': cmd, 'fixture_sha256': sha(target/'fixture'), 'environment': env}


def records(path, mask_lio_wall=False):
    result = []
    with Path(path).open('rb') as f:
        require(f.read(16) == b'FLIVODIAG0001LE\0', 'Foreign diagnostic header')
        while header := f.read(56):
            require(len(header) == 56, 'Truncated diagnostic header')
            h = struct.unpack('<7Q', header)
            require(h[-1] <= 4194304, 'Unexpected diagnostic payload')
            data = f.read(h[-1]*8); require(len(data) == h[-1]*8, 'Truncated diagnostic payload')
            if mask_lio_wall and h[0] == 100:
                require(h[-1] >= 13, 'Missing documented kind100 wall slots')
                data = data[:9*8]+bytes(4*8)+data[13*8:]
            result.append((h, data))
    return result


def vio_cases():
    cases = []
    settings = [(0,0,0,0,0,0,0,0,0,'zero'), (9,0,0,0,0,0,0,0,0,'small9'),
        (16,2,0,1,1,0,0,0,0,'small_exposure'), (128,1,1,1,0,0,0,0,0,'nullable'),
        (128,2,1,1,1,0,0,0,0,'nullable_exposure'), (128,2,0,0,0,0,0,0,1,'all_null'),
        (128,1,0,0,0,1,0,0,0,'fresh_cache'), (128,1,0,0,1,1,0,0,0,'fresh_exposure'),
        (128,0,0,0,0,0,1,0,0,'multi_pyramid'), (128,0,0,1,1,0,1,0,0,'multi_exposure'),
        (512,1,0,1,0,0,0,0,0,'large512')]
    for mode in ('forward', 'inverse'):
        for n, level, null, mixed, expo, fresh, multi, wrong, allnull, name in settings:
            cases.append((mode+'_'+name, [mode, *map(str, (n,level,null,mixed,expo,fresh,multi,wrong,allnull)), '1']))
        for level in (0,1,2):
            for expo in (0,1): cases.append((mode+f'_level{level}_exposure{expo}', [mode,'128',str(level),'0','1',str(expo),'0','0','0','0','1']))
        for n in (63,64,65): cases.append((mode+f'_threshold_{n}', [mode,str(n),'0','0','0','0','0','0','0','0','1']))
    cases.append(('inverse_rollback_probe', ['inverse','128','0','0','0','0','0','0','1','0','1']))
    require(len(cases) == 41, 'Wrong independent VIO coverage')
    return cases


def run_one(build, tag, args, diagnostics):
    d = Path(build['target'])/tag; d.mkdir(exist_ok=False)
    env = dict(build['environment'])
    env.update(LD_PRELOAD=str(HERE/'observe_fp_team_loader.so'), V15_GOMP_RECORDS=str(d/'teams.jsonl'),
               V15_MAIN_FP_RECORDS=str(d/'main_FP.jsonl'), V19_LOADED_LIBRARY_MAPS=str(d/'loaded_maps.txt'))
    if diagnostics != 'off':
        env.update(FASTLIVO_DIAGNOSTIC_DIR=str(d/'diagnostics'),
                   FASTLIVO_DIAGNOSTIC_BEGIN='129' if diagnostics == 'inside' else '115',
                   FASTLIVO_DIAGNOSTIC_END='131' if diagnostics == 'inside' else '118')
    cmd = ['taskset','-c',AFFINITY,str(Path(build['target'])/'fixture'),*args,str(d/'output.bin')]
    r = subprocess.run(cmd, env=env, text=True, capture_output=True)
    (d/'stdout.log').write_text(r.stdout); (d/'stderr.log').write_text(r.stderr)
    require(r.returncode == 0, 'Fresh run failed '+str(d)+': '+r.stderr[-1500:])
    output = d/'output.bin'; require(output.is_file() and output.stat().st_size > 0, 'Missing numerical output')
    fp = [json.loads(line) for line in (d/'main_FP.jsonl').read_text().splitlines()]
    fp_actual = [row for row in fp if Path(row['executable']).resolve() == (Path(build['target'])/'fixture').resolve()]
    require([row['phase'] for row in fp_actual] == ['main_enter','main_exit'], 'No complete actual fixture FP witness')
    require(all(row['fp_rounding_mode'] == 0 and row['mxcsr_control_mask'] == 8064 and row['cpu'] in (0,2,4,6) for row in fp_actual), 'Changed actual FP or P-core affinity')
    teams = [json.loads(line) for line in (d/'teams.jsonl').read_text().splitlines()]
    require(all(all(c in (0,2,4,6) for c in row['cpus']) and all(c == 0 for c in row['fp_rounding_modes']) and all(c == 8064 for c in row['mxcsr_control_masks']) for row in teams), 'Worker FP/affinity mismatch')
    maps = (d/'loaded_maps.txt').read_text().splitlines()
    loaded_paths = {line.split()[-1] for line in maps if 'libfast_livo2_core.so' in line and line[0].isalnum()}
    require(loaded_paths == {str(Path(build['library']).resolve())}, 'Missing/foreign actual loaded core')
    stats = None
    if diagnostics != 'off':
        stats = json.loads((d/'diagnostics/writer_stats.json').read_text())
        require(stats['final'] is True and stats['dropped'] == 0 and stats['writer_io_failed'] is False and stats['attempted'] == stats['written'], 'Incomplete fresh diagnostic writer')
    return {'argv': cmd, 'returncode': 0, 'output': str(output), 'output_sha256': sha(output),
            'actual_main_FP': fp_actual, 'actual_teams': teams, 'actual_loaded_core_paths': sorted(loaded_paths),
            'diagnostics': diagnostics, 'writer_stats': stats}


def main():
    receipt_path = HERE/'FRESH_MATH_FP_TEAM_LOADER_RECEIPT.json'
    require(not receipt_path.exists(), 'Refuse existing receipt')
    observer_cmd = ['/usr/bin/c++','-std=c++17','-O2','-shared','-fPIC','-fopenmp',
                    str(HERE/'observe_fp_team_loader.cpp'),'-ldl','-o',str(HERE/'observe_fp_team_loader.so')]
    obs = subprocess.run(observer_cmd, capture_output=True, text=True)
    (HERE/'observer_compile.log').write_text(obs.stdout+obs.stderr); require(obs.returncode == 0, 'Observer compile failed')
    tasks = [(variant,HERE/source,name) for variant in VARIANTS for source,name in
             (('finite_lio_vio_fixture.cpp','lio_vio9'),('vio_patch_fixture.cpp','vio41'))]
    builds = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        futures = [executor.submit(compile_fixture,*task) for task in tasks]
        for future in futures: builds.append(future.result())
    with (HERE/'FRESH_BUILD_RECEIPT.json').open('x') as f: json.dump(builds,f,indent=2); f.write('\n')
    mapping = {(b['variant'],b['name']): b for b in builds}; runs = {}; comparisons = []
    for variant in VARIANTS:
        runs[variant] = {}
        for scene in ('lio','vio','inverse'):
            for diagnostic in ('off','inside','outside'):
                tag = scene+'_'+diagnostic
                runs[variant][tag] = run_one(mapping[(variant,'lio_vio9')],tag,[scene],diagnostic)
        for tag,args in vio_cases():
            runs[variant][tag] = run_one(mapping[(variant,'vio41')],tag,args,'inside')
    for scene in ('lio','vio','inverse'):
        entries = [runs[variant][scene+'_'+diagnostic] for variant in VARIANTS for diagnostic in ('off','inside','outside')]
        byte_equal = len({Path(e['output']).read_bytes() for e in entries}) == 1
        diag_a = records(Path(runs['baseline_V18'][scene+'_inside']['output']).parent/'diagnostics/records.bin', True)
        diag_b = records(Path(runs['candidate_V19'][scene+'_inside']['output']).parent/'diagnostics/records.bin', True)
        comparisons.append({'case': '729_or_9_'+scene, 'six_outputs_byte_identical': byte_equal,
                            'all_nonwall_inside_records_byte_identical': diag_a == diag_b,
                            'wall_mask': 'kind100 float64 positions9..12 only', 'records': len(diag_a)})
    accepted, rollback = 0, 0
    for tag, _ in vio_cases():
        a, b = (runs[v][tag] for v in VARIANTS)
        da = records(Path(a['output']).parent/'diagnostics/records.bin')
        db = records(Path(b['output']).parent/'diagnostics/records.bin')
        for h,payload in da:
            if h[0] == 201 and len(payload) > 12*8:
                flag = struct.unpack_from('<d',payload,9*8)[0]
                accepted += flag == 1.; rollback += flag == 0.
        comparisons.append({'case': tag, 'state_cov_G_H_errors_reference_H_byte_identical': Path(a['output']).read_bytes() == Path(b['output']).read_bytes(),
                            'all_nonwall_records_byte_identical': da == db, 'records': len(da)})
    numeric_ok = all(all(v for k,v in row.items() if 'byte_identical' in k) for row in comparisons)
    team_lio = [row['actual_team'] for variant in VARIANTS for row in runs[variant]['lio_off']['actual_teams'] if 'VoxelMapManager' in row['caller']]
    require(bool(team_lio) and set(team_lio) == {4}, 'Actual LIO4 team not witnessed')
    result = {'schema':'go2_V19_fresh_finite_math_FP_team_loader/v1',
              'status':'PASS_LIMITED_FRESH_KERNEL_REGRESSION' if numeric_ok else 'FAILED',
              'fresh_process_runs': sum(len(v) for v in runs.values()), 'VIO_distinct_cases':41,
              'comparisons':comparisons, 'accepted_iterations':accepted, 'rollback_iterations':rollback,
              'builds': builds, 'runs':runs, 'observer_source_sha256':sha(HERE/'observe_fp_team_loader.cpp'),
              'observer_library_sha256':sha(HERE/'observe_fp_team_loader.so'), 'observer_argv':observer_cmd,
              'observer_is_test_only':True,'team_override':False,'FP_mode_changed_by_observer':False,
              'full_processFrame_or_actual_corpus_equivalence':'UNVERIFIED',
              'full_pipeline_ingress_lifecycle_equivalence':'NOT_TESTED_HERE',
              'numeric_scope':'LIO729 + VIO9 forward/inverse diagnostic off/inside/outside;41 synthetic patch update cases',
              'null_precompute_limit':'Inverse null fixtures use existing prepared H cache; original inverse fresh precompute dereferences null before guard'}
    with receipt_path.open('x') as f: json.dump(result,f,indent=2,allow_nan=False); f.write('\n')
    print(json.dumps({'status':result['status'],'fresh_process_runs':result['fresh_process_runs'],'receipt':str(receipt_path)}))
    return 0 if numeric_ok else 1


if __name__ == '__main__': raise SystemExit(main())
