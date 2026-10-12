"""V33 reuses the frozen independently-built V32 SLAM workspace read-only."""
from pathlib import Path
import importlib.util
import sys

BASELINE=Path(__file__).resolve().parent.parent/'map_surface_v32'
spec=importlib.util.spec_from_file_location('_v33_frozen_v32_workspace',BASELINE/'slam_workspace.py')
baseline=importlib.util.module_from_spec(spec);sys.modules[spec.name]=baseline;spec.loader.exec_module(baseline)
SLAM_WORKSPACE=baseline.SLAM_WORKSPACE
CONTRACT=baseline.CONTRACT
CORE_RECEIPT=baseline.CORE_RECEIPT
sha=baseline.sha
required_workspace_files=baseline.required_workspace_files
verify_slam_workspace=baseline.verify_slam_workspace

def evidence_files():
    return [Path(__file__).resolve(),BASELINE/'slam_workspace.py',*baseline.evidence_files()]
