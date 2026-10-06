"""Bound recording growth without deleting or signaling another task's files."""
from pathlib import Path
import os
import time


def observe(run,profile,starting=False,count_files=False):
    run=Path(run).resolve();policy=profile['raw_storage_budget']
    if set(policy)!={'max_run_bytes','minimum_start_available_bytes','minimum_runtime_available_bytes'}:
        raise RuntimeError('Explicit V19 recording budget required')
    if any(type(x)is not int or x<=0 for x in policy.values()):
        raise RuntimeError('Invalid recording budget')
    stats=os.statvfs(run);available=stats.f_bavail*stats.f_frsize
    result={'monotonic_wall':time.monotonic(),'available_bytes':available,'budget':policy,
            'size_check_interval_wall_s':15,'size_is_logical_bytes_not_allocated_blocks':True}
    minimum=policy['minimum_start_available_bytes']if starting else policy['minimum_runtime_available_bytes']
    if available<minimum:raise RuntimeError('V19 recording reserve insufficient; only this run may stop')
    if count_files:
        total=0;count=0
        for base,dirs,names in os.walk(run,followlinks=False):
            dirs[:]=[n for n in dirs if not Path(base,n).is_symlink()]
            for name in names:
                p=Path(base,name)
                if p.is_symlink():continue
                try:total+=p.stat().st_size;count+=1
                except FileNotFoundError:continue
        result.update(observed_run_bytes=total,observed_files=count)
        if total>policy['max_run_bytes']:
            raise RuntimeError('V19 recording budget exceeded; retain failure and stop only this run')
    return result
