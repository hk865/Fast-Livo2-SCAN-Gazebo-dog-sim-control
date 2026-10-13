def stop_owned(proc,saved,parent_first=False):
    if proc is None:return {'started':False}
    if saved is None:return {'started':True,'identity_unavailable':True,'sent_signals':[],'returncode':proc.poll()}
    sent=[];parent=identity(proc.pid)
    if saved.get('ownership_token'):
        from owned_guardian import validated_members
        validated_members(saved)
    if parent and parent['start_ticks']!=saved['start_ticks']:
        return {'identity_mismatch':True,'sent_signals':[],'returncode':proc.poll()}
    for sig,timeout in ((signal.SIGINT,15 if parent_first else 5),(signal.SIGTERM,5),(signal.SIGKILL,3)):
        if not group_members(saved['pgid']):break
        if saved.get('ownership_token'):validated_members(saved)
        try:
            if parent_first and sig==signal.SIGINT and proc.poll()is None:trace_signal('pid',proc.pid,sig)
            else:trace_signal('pgid',saved['pgid'],sig)
            sent.append(int(sig));until=time.monotonic()+timeout
            while time.monotonic()<until:
                proc.poll()
                if not group_members(saved['pgid']):break
                time.sleep(.1)
        except ProcessLookupError:break
    proc.poll()
    return {'started':True,'sent_signals':sent,'returncode':proc.returncode,'remaining_owned_group_members':group_members(saved['pgid'])}
