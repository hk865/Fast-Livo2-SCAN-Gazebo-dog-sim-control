"""Recheck original faults after worker EOF; no completion or navigation authority."""
from first_goal_stop import observe_native_tail, first_goal_runtime_decision

def terminal_original_decision(run,tail,previous_ns,wall_elapsed,output_bytes):
    observed=observe_native_tail(tail,previous_ns)
    decision=first_goal_runtime_decision(run,observed,wall_elapsed,output_bytes)
    if not decision['stop'] and not observed['caught_up']:
        decision=dict(stop=True,reason='R77_terminal_native_backlog_unverified',navigation_PASS=False,
            diagnostic_only=True,original_route_acceptance=False)
    return observed,decision
