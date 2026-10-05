"""Capture owned process paths before shutdown, then verify after physics exits."""
import hashlib
import json
import os
from pathlib import Path
import time


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2)+'\n')


def capture_before_stop(directory, group):
    """Only known startup PIDs; no hashing or ROS work while physics is active."""
    directory = Path(directory)
    result = {'owned_process_group': group, 'captured_wall_monotonic_ns': time.monotonic_ns()}
    for role, filename in [('gazebo', 'startup_control_runtime.json'), ('gait', 'startup_gait_runtime.json')]:
        try:
            startup = read(directory/filename)
            pid = startup['pid'] if role == 'gazebo' else startup['actual'][0]['pid']
            proc = Path('/proc')/str(pid)
            if os.getpgid(pid) != group:
                raise ValueError('startup process no longer belongs to this run')
            row = {'pid': pid, 'owned_process_group': group,
                   'executable': str((proc/'exe').resolve(strict=True)),
                   'raw_maps': (proc/'maps').read_text(),
                   'captured_wall_monotonic_ns': time.monotonic_ns()}
            if role == 'gait':
                environ = dict(x.split('=', 1) for x in (proc/'environ').read_bytes().decode().split('\0') if '=' in x)
                row['environment'] = {key: environ.get(key) for key in ('LD_LIBRARY_PATH','AMENT_PREFIX_PATH','ROS_DOMAIN_ID')}
                from simulation.gait_runtime import mapped_paths
                row['libraries'] = [{'path': p} for p in mapped_paths(row['raw_maps']) if '.so' in Path(p).name]
            result[role] = row
        except (OSError, ValueError, KeyError, IndexError) as exc:
            result[role] = {'error': repr(exc)}
    write(directory/'prestop_runtime_paths.json', result)
    return result


def verify_after_stop(directory, root, group):
    """Missing startup/pre-stop evidence fails; historical contracts are unchanged."""
    from mission.processes import group_running
    from simulation.capture_control_runtime import verify as verify_gazebo
    from simulation.gait_runtime import verify as verify_gait
    directory, root = Path(directory), Path(root)
    checks = {'owned_group_clean': not group_running(group)}
    details = {}
    try:
        manifest = read(directory/'runtime_manifest.json')
        for name in ('controller_ready_gate', 'control_parameter_readback', 'startup_control_runtime', 'startup_gait_runtime', 'startup_gate'):
            item = read(directory/(name+'.json'))
            checks[name+'_passed'] = item.get('passed') is True
        paths = read(directory/'prestop_runtime_paths.json')
        checks['prestop_group_matches'] = paths['owned_process_group'] == group
        actual, gzchecks = verify_gazebo(paths['gazebo']['raw_maps'], manifest['artifacts'])
        checks['prestop_gazebo_libraries_match'] = all(gzchecks.values())
        from simulation.capture_control_runtime import paths_from_maps
        plugin = manifest['artifacts']['gazebo_measured_joint_plugin']
        checks['actual_measured_joint_plugin_mapped'] = plugin['path'] in paths_from_maps(paths['gazebo']['raw_maps'])
        startup = read(directory/'startup_gait_runtime.json')
        gait = verify_gait([paths['gait']], manifest['selected_gait'], startup['inherited_environment'])
        checks['prestop_gait_libraries_match'] = gait['passed']
        checks['same_startup_terminal_gazebo_pid'] = paths['gazebo']['pid'] == read(directory/'startup_control_runtime.json')['pid']
        checks['same_startup_terminal_gait_pid'] = paths['gait']['pid'] == startup['actual'][0]['pid']
        sources = read(directory/'source_manifest.json')['sha256']
        checks['all_frozen_sources_unchanged'] = all(hashlib.sha256((root/name).read_bytes()).hexdigest() == digest for name,digest in sources.items())
        checks['all_runtime_artifacts_unchanged'] = all(hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() == item['sha256'] for item in manifest['artifacts'].values())
        selected = manifest['selected_gait']
        checks['all_36_selected_gait_sources_unchanged'] = len(selected['source_sha256']) == 36 and all(hashlib.sha256((Path(selected['source_root'])/name).read_bytes()).hexdigest() == digest for name,digest in selected['source_sha256'].items())
        checks['selected_gait_provenance_unchanged'] = all(hashlib.sha256(Path(selected[name]['path']).read_bytes()).hexdigest() == selected[name]['sha256'] for name in ('frozen_build_receipt','frozen_source_archive_receipt','frozen_patch','deployment_manifest'))
        details.update(gazebo={'actual_libraries': actual,'checks': gzchecks}, gait=gait)
    except (OSError, ValueError, KeyError, IndexError) as exc:
        checks['complete_runtime_evidence'] = False
        details['error'] = repr(exc)
    result = {'passed': all(checks.values()), 'checks': checks, 'details': details,
              'scope': 'Actual startup and pre-stop runtime, source integrity and owned cleanup; no route acceptance inferred'}
    write(directory/'full_control_runtime_evidence.json', result)
    return result
