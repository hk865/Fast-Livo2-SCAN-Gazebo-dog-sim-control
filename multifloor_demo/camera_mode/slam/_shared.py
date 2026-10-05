"""Explicit, checked reuse of the existing isolated sensor-SLAM sources.

This module imports no ROS runtime and creates no nodes. The old quadruped
launcher, filter, evaluator entry point and controller prerequisites are unused.
"""
import hashlib
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
CONTRACT = HERE / 'source_contract.json'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_sources(include_binaries=False):
    contract = json.loads(CONTRACT.read_text())
    selected = dict(contract['sources'])
    if include_binaries:
        selected.update(contract['isolated_binaries'])
    errors = [name for name, expected in selected.items()
              if not (ROOT / name).is_file() or sha256(ROOT / name) != expected]
    if errors:
        raise RuntimeError('Camera SLAM source/runtime contract changed: ' + ', '.join(errors))
    return contract


def load_shared(name, relative_path):
    verify_sources()
    spec = importlib.util.spec_from_file_location('_camera_slam_shared_' + name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def snapshot_contract(run_dir):
    """Record the real selected core and camera settings before starting SLAM."""
    contract = verify_sources(include_binaries=True)
    out = Path(run_dir) / 'camera_slam_contract.json'
    document = dict(contract, mode='camera', ground_truth_used=False,
                    self_echo_filter='disabled: no Go2 body/leg mask',
                    camera_files={p.name: sha256(p) for p in sorted(HERE.glob('*'))
                                  if p.is_file() and p.suffix in ('.py', '.yaml', '.json')})
    if out.exists() and json.loads(out.read_text()) != document:
        raise RuntimeError('Run already contains a different camera SLAM contract')
    out.write_text(json.dumps(document, indent=2) + '\n')
    return document
