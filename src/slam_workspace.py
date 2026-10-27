"""Project-local native SLAM workspace; no source/build/version contract."""
from pathlib import Path
from project_config import path
SLAM_WORKSPACE=path('native_workspace')
SLAM_SOURCE_ROOT=__import__('project_config').ROOT/'native/slam/src'
PRIVATE=True
def required_workspace_files():
    return [SLAM_WORKSPACE/'install/local_setup.bash',SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so']
def verify_slam_workspace():
    for p in required_workspace_files():
        if not p.is_file():raise FileNotFoundError('Native SLAM dependency missing: '+str(p))
    return dict(workspace=str(SLAM_WORKSPACE),source_root=str(SLAM_SOURCE_ROOT),mode='serial')
def evidence_files():return required_workspace_files()
