"""Only two explicitly frozen SCAN workspaces can supply the V21 runtime (inherited exact V20 workspace)."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent
SLOTS = {
    'protected_baseline': (HERE.parents[2]/'navigation/ros2_ws', 'SCAN_BASELINE_WORKSPACE_CONTRACT.json'),
    'bounded_snapshot_exporter_v20': (HERE.parent/'corridor_tracking_v20/scan_ws', 'SCAN_CANDIDATE_WORKSPACE_CONTRACT.json'),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scan_contract(profile=None):
    selector = (profile or {}).get('scan_workspace_selector', 'protected_baseline')
    if selector not in SLOTS:
        raise RuntimeError('An arbitrary or unreviewed SCAN workspace is prohibited')
    workspace, name = SLOTS[selector]
    contract = HERE/name
    if not contract.is_file():
        raise RuntimeError('Selected SCAN workspace has no frozen source/build contract')
    d = json.loads(contract.read_text())
    if (d.get('schema') != 'corridor_v20_SCAN_workspace/v1' or d.get('candidate') != selector
            or d.get('workspace') != str(workspace.resolve()) or d.get('algorithm_math_changed') is not False
            or d.get('runtime_acceptance_inherited') is not False
            or d.get('actual_loaded_binary_witness_required') is not True):
        raise RuntimeError('The selected SCAN contract identity differs')
    required = [workspace/'install/setup.bash', workspace/'install/local_setup.bash',
        workspace/'install/scan_planner/lib/scan_planner/scan_planner_node',
        workspace/'install/scan_planner/share/scan_planner/config/planner.yaml']
    bindings = d.get('bindings_sha256')
    if not isinstance(bindings, dict) or any(str(p.resolve()) not in bindings for p in required):
        raise RuntimeError('Mandatory SCAN runtime bindings are missing')
    for path, digest in bindings.items():
        if not Path(path).is_absolute() or sha(path) != digest:
            raise RuntimeError('Frozen SCAN source/build/runtime changed: ' + path)
    return workspace.resolve(), contract, d


def evidence_files(profile=None):
    _, contract, d = scan_contract(profile)
    return [contract, *[Path(name) for name in d['bindings_sha256']]]
