#!/usr/bin/env python3
"""Offline forward-check of one actual JTC trace. No GT, ROS, or actuator output."""
import hashlib
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
DEMO=HERE.parents[2]
INPUT=DEMO/"simulation/test_results/20261001_default_startup_actuator_v1/jtc_decoded_13_to_17.jsonl"
HORIZON_NS=16666666
NEXT_SAMPLE_NS=4000000


def replay(rows):
    previous=None
    start_effort=None
    previous_pid=None
    previous_integral=None
    initialized=None
    residuals={"positive_dt":[],"zero_dt":[]}
    adopted=0
    step_rows=[]
    for index,row in enumerate(rows):
        if previous is None:
            previous=row
            continue
        dt_ns=row["meta"]["header_stamp_ns"]-previous["meta"]["header_stamp_ns"]
        if dt_ns < 0:
            raise ValueError("Backward controller header clock requires another model; never silently coerce")
        phase_ns=row["reference"]["time_from_start_ns"]
        fresh=(phase_ns==0 and row["reference"]["positions"]==row["feedback"]["positions"])
        if fresh:
            start_effort=previous["output"]["effort"]
            adopted+=1
        if start_effort is None:
            previous=row
            continue
        factor=max(0.,1.-(phase_ns+NEXT_SAMPLE_NS)/HORIZON_NS)
        ff=[factor*v for v in start_effort]
        actual=row["output"]["effort"]
        e=row["error"]["positions"]
        edot=row["error"]["velocities"]
        if previous_integral is None:
            # Exactly one initial condition per joint; not a per-step fit.
            if dt_ns==0:
                previous=row
                continue
            previous_integral=[u-f-100*p-d for u,f,p,d in zip(actual,ff,e,edot)]
            previous_pid=[u-f for u,f in zip(actual,ff)]
            initialized={"row":index,"stamp_ns":row["meta"]["header_stamp_ns"],"integral":previous_integral.copy()}
            previous=row
            continue
        if dt_ns==0:
            predicted_pid=previous_pid.copy()
            integral=previous_integral.copy()
        else:
            integral=[max(-2.5,min(2.5,i+.2*p*dt_ns/1e9)) for i,p in zip(previous_integral,e)]
            predicted_pid=[100*p+d+i for p,d,i in zip(e,edot,integral)]
        predicted=[f+p for f,p in zip(ff,predicted_pid)]
        errors=[abs(a-b) for a,b in zip(actual,predicted)]
        kind="zero_dt" if dt_ns==0 else "positive_dt"
        residuals[kind].extend(errors)
        step_rows.append({"row":index,"stamp_ns":row["meta"]["header_stamp_ns"],"dt_ns":dt_ns,
                          "fresh_reference_from_feedback":fresh,"carry_factor":factor,
                          "max_abs_output_residual":max(errors)})
        previous_pid=predicted_pid
        previous_integral=integral
        previous=row
    summarize=lambda a:{"components":len(a),"max_abs_residual":max(a),"p99_abs_residual":sorted(a)[math.ceil(.99*len(a))-1]}
    return {"actual_jtc_rows":len(rows),"forward_steps":len(step_rows),
            "positive_dt_steps":sum(r["dt_ns"]>0 for r in step_rows),
            "zero_dt_steps":sum(r["dt_ns"]==0 for r in step_rows),
            "adopted_reference_starts_after_first_row":adopted,
            "one_time_initial_condition":initialized,"residuals":{k:summarize(v) for k,v in residuals.items()},
            "all_output_components_agree_within_1e_minus_10":all(max(v)<1e-10 for v in residuals.values()),
            "steps":step_rows,
            "limits":["This is the separate passive startup probe, not the original failed full16 run.",
                      "Header differences reproduce the period-dependent behavior; internal controller period was not directly instrumented.",
                      "The initial integral is inferred once from one actual output. All following outputs are forward predictions without re-fitting.",
                      "Published effort is the command interface, not independently measured physical applied torque.",
                      "No GT, no-slip assumption, or body-state fit enters this replay."]}


def main():
    data=INPUT.read_bytes()
    rows=[json.loads(line) for line in data.splitlines()]
    result=replay(rows)
    result["input"]={"path":str(INPUT),"sha256":hashlib.sha256(data).hexdigest()}
    result["source_model"]={"p":100,"i":.2,"d":1,"integral_bounds":[-2.5,2.5],
                            "position_only_horizon_ns":HORIZON_NS,"next_sample_ns":NEXT_SAMPLE_NS,
                            "zero_dt":"retain previous PID command; keep trajectory time frozen",
                            "effort_feedforward":"interpolate previous command effort to endpoint zero over the same horizon"}
    (HERE/"jtc_output_forward_replay.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k in ("actual_jtc_rows","forward_steps","positive_dt_steps","zero_dt_steps","residuals","all_output_components_agree_within_1e_minus_10")},indent=2))


if __name__=="__main__":
    main()
