"""Preserve the exact demo sources and configuration used by a run."""
import hashlib
import json
import os
from pathlib import Path
import tarfile


def snapshot_sources(root, destination):
    root, destination = Path(root), Path(destination)
    extensions = {'.py', '.sh', '.yaml', '.yml', '.json', '.html', '.sdf', '.urdf',
                  '.cpp', '.cc', '.h', '.hpp', '.xml', '.cmake', '.msg', '.srv'}
    files = []
    for folder in ('mission', 'scripts', 'navigation', 'slam', 'simulation', 'web'):
        files.extend(p for p in (root/folder).rglob('*') if p.is_file()
                     and (p.suffix in extensions or p.name in {'CMakeLists.txt', 'LICENSE', 'LICENSE.txt', 'NOTICE', 'COPYING'}
                         or (root/'simulation/gait_control_ws/src') in p.parents)
                     and not any(part.lower() in {'test_results', '__pycache__', 'runs', 'build', 'install', 'log', 'logs', 'output', '.git'}
                                 for part in p.relative_to(root).parts))
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    with tarfile.open(destination/'source_snapshot.tar.gz', 'w:gz') as archive:
        for p in sorted(files):
            archive.add(p, arcname=str(p.relative_to(root)), recursive=False)
    (destination/'source_manifest.json').write_text(json.dumps({'schema_version': 1, 'sha256': hashes}, indent=2)+'\n')


def snapshot_runtime(destination, *, control_timing_enabled=False):
    """Record the resolved installed artifacts, including the isolated SLAM core."""
    from ament_index_python.packages import get_package_prefix
    artifacts = {}
    for package, relative in (
        ('fast_livo2_ros', 'lib/fast_livo2_ros/fastlivo_mapping'),
        ('fast_livo2_core', 'lib/libfast_livo2_core.so'),
        ('scan_planner', 'lib/scan_planner/scan_planner_node'),
        ('champ_base', 'lib/champ_base/quadruped_controller_node'),
        ('champ_base_controller', 'lib/libquadruped_controller.so'),
        ('controller_manager', 'lib/libcontroller_manager.so'),
        ('controller_interface', 'lib/libcontroller_interface.so'),
        ('control_toolbox', 'lib/libcontrol_toolbox.so'),
        ('joint_trajectory_controller', 'lib/libjoint_trajectory_controller.so'),
        ('joint_state_broadcaster', 'lib/libjoint_state_broadcaster.so'),
        ('gz_ros2_control', 'lib/libgz_ros2_control-system.so'),
    ):
        lookup_package = 'champ_base' if package == 'champ_base_controller' else package
        if package == 'controller_manager':
            from simulation.controller_runtime import controller_manager_library
            path = controller_manager_library()
        else:
            path = Path(get_package_prefix(lookup_package))/relative
        artifacts[package] = {'path': str(path.resolve()),
                              'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    for package in ('plan_env', 'path_searching', 'bspline_opt'):
        library_dir = Path(get_package_prefix(package))/'lib'
        for path in sorted(library_dir.glob('lib'+package+'.*')):
            if path.suffix in {'.a', '.so'}:
                artifacts[package+path.suffix] = {
                    'path': str(path.resolve()),
                    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    baseline_gait_artifacts = {key: dict(artifacts[key]) for key in ('champ_base', 'champ_base_controller')}
    from simulation.gait_selection import snapshot_selected_gait
    selected_gait = snapshot_selected_gait(destination, artifacts)
    artifacts['champ_base'] = dict(selected_gait['executable'])
    artifacts['champ_base_controller'] = dict(selected_gait['library'])
    measured_plugin = Path('/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-joint-state-publisher-system.so.8.11.0')
    artifacts['gazebo_measured_joint_plugin'] = {'path': str(measured_plugin), 'sha256': hashlib.sha256(measured_plugin.read_bytes()).hexdigest()}
    from simulation.execution_safety import adapter_settings
    from simulation.controller_runtime import controller_timing_configuration
    from simulation.control_timing_trace import timing_audit_configuration
    manifest = {'schema_version': 2, 'artifacts': artifacts,
                'selected_gait': selected_gait, 'baseline_gait_artifacts': baseline_gait_artifacts,
                'full_control_runtime_required': True,
                'joint_stop_adapter': adapter_settings(),
                'controller_timing_configuration': controller_timing_configuration(),
                'control_timing_diagnostics': timing_audit_configuration(control_timing_enabled),
                'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', 72)),
                'gazebo_partition': os.environ.get('GZ_PARTITION', '')}
    (Path(destination)/'runtime_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
