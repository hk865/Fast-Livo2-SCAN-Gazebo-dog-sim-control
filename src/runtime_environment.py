"""Explicit ROS/SDK environment for this package; no retired overlay sourcing."""
from pathlib import Path
import json
import os
import subprocess

PACKAGE = Path(__file__).resolve().parent.parent
RUNTIME = PACKAGE / '.local/runtime'
NATIVE = RUNTIME / 'native/ws/install'
SCAN = RUNTIME / 'protocol_sender/protocol_sender/scan_ws/install'
SDK_NAMES = ('scan_planner_msgs', 'livox_ros_driver2', 'champ_msgs', 'vikit_common', 'vikit_ros')


def _prefix_paths(prefix):
    prefix = Path(prefix).resolve(strict=True)
    if not prefix.is_relative_to(PACKAGE):
        raise RuntimeError('Runtime prefix escapes this package: '+str(prefix))
    return prefix


def environment(ros_base, *, scan_overlay=None):
    """Keep ROS and the selected SDKs, with explicit project-local resolution.

    No generated setup.bash underlay chain is sourced. The local setup files
    retain ordinary ROS hooks; SDK imports and shared libraries are enumerated
    independently. Only path variables are returned in diagnostics elsewhere.
    """
    native = _prefix_paths(NATIVE)
    scan = _prefix_paths(SCAN)
    selected = scan/'local_setup.bash' if scan_overlay is None else Path(scan_overlay).resolve(strict=True)
    if selected not in {scan/'local_setup.bash', scan/'setup.bash'}:
        raise RuntimeError('Unbound SCAN setup selection: '+str(selected))
    # Never replay the generated setup.bash's historical underlay sequence.
    selected = scan/'local_setup.bash'
    child = dict(os.environ)
    for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                'COLCON_PREFIX_PATH', 'GZ_SIM_RESOURCE_PATH', 'GAZEBO_MODEL_PATH'):
        child.pop(key, None)
    # ROS supplies its own tools; project binaries enter only through package
    # local_setup hooks. Preserve user/system tools outside the project tree.
    project = str(PACKAGE.parent)+'/'
    child['PATH'] = ':'.join(p for p in child.get('PATH', '/usr/bin:/bin').split(':')
                             if p and not p.startswith(project))
    command = ['bash', '-c',
        'set -e; source "$1" >/dev/null; source "$2" >/dev/null; source "$3" >/dev/null; env -0',
        'independent-runtime-environment', str(ros_base), str(native/'local_setup.bash'), str(selected)]
    result = subprocess.run(command, check=True, capture_output=True, env=child)
    value = dict(item.decode().split('=', 1) for item in result.stdout.split(b'\0') if b'=' in item)
    sdk = [_prefix_paths(RUNTIME/'sdk'/name) for name in SDK_NAMES]
    libraries = [native/'fast_livo2_core/lib', *(p/'lib' for p in sdk)]
    python = [p/'lib/python3.12/site-packages' for p in sdk]
    ament = [native/'fast_livo2_ros', native/'fast_livo2_core', scan/'scan_planner', *sdk]
    for key, paths in [('LD_LIBRARY_PATH', libraries), ('PYTHONPATH', python), ('AMENT_PREFIX_PATH', ament)]:
        leading = [str(p) for p in paths if p.is_dir()]
        value[key] = ':'.join(dict.fromkeys([*leading, *filter(None, value.get(key, '').split(':'))]))
    # No inherited project prefix, symlinked setup, or old SDK can survive.
    for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for entry in filter(None, value.get(key, '').split(':')):
            path = Path(entry).resolve()
            if str(path).startswith(project) and not path.is_relative_to(PACKAGE):
                raise RuntimeError('Retired project path in '+key+': '+str(path))
    return value
