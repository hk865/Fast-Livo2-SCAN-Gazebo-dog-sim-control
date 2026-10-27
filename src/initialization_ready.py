"""History availability only; original initialization verifier retains authority."""

def initialization_history_ready(poses):
    return (len(poses)>=10
        and poses[-1]['stamp_ns']-poses[0]['stamp_ns']>=2_900_000_000)
