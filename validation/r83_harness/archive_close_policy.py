"""Bound archive waiting after verified native shutdown.

This does not authorize waiting while physics runs. The original
await_archive_close still verifies Gazebo stop, worker identity and ownership.
Failed pipeline/capture/worker evidence remains failed in the original manifest.
"""


def post_physics_archive_abort_reason(cleanup, *, watchdog_error, guardian_error,
                                     now, wall_deadline):
    if watchdog_error:
        return 'storage watchdog: ' + str(watchdog_error)
    if guardian_error:
        return 'owned guardian: ' + str(guardian_error)
    if now >= wall_deadline:
        return 'original frozen wall deadline'
    for role, result in cleanup.items():
        if (result.get('error') or result.get('identity_mismatch')
                or result.get('identity_unavailable')
                or result.get('remaining_owned_group_members')):
            return 'original owned role cleanup incomplete: ' + role
        if result.get('started') is False:
            continue
        if (result.get('started') is not True
                or result.get('returncode') is None
                or result.get('remaining_owned_group_members') != []):
            return 'original owned role cleanup incomplete: ' + role
    # Once these groups are absent, an original nonzero returncode or failed
    # pipeline cannot produce more physics. Neither becomes a successful run.
    return None
