"""Explicit R80 current-retained guard proxy; no historical fallback."""
from pathlib import Path
import hashlib,importlib.util,sys
S=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/.local_dependencies/storage_guard')
PATH=S/'storage_guard.py'
if hashlib.sha256(PATH.read_bytes()).hexdigest()!='ebf29f18def24fe32955c93547ebd29d8cc989ded71acdb79bf2e040f91a5717':
    raise RuntimeError('R80 approved current-retained guard source changed')
sys.path.insert(0,str(S))
spec=importlib.util.spec_from_file_location('R80_current_retained_guard',PATH)
base=importlib.util.module_from_spec(spec);sys.modules[spec.name]=base;spec.loader.exec_module(base)
register,usage,capacity=base.register,base.usage,base.capacity
legacy_observe=base.observe
def observe(run,profile,**kwargs):
    if profile.get("r85_session",{}).get("enabled"):
        from r85_storage_guard import observe as fresh_observe
        return fresh_observe(run,profile,**kwargs)
    return legacy_observe(run,profile,**kwargs)
StorageStop=base.StorageStop
class Watchdog:
    def __new__(cls,run,profile,stop_runner):
        if profile.get("r85_session",{}).get("enabled"):
            from r85_storage_guard import Watchdog as FreshWatchdog
            return FreshWatchdog(run,profile,stop_runner)
        return base.Watchdog(run,profile,stop_runner)
