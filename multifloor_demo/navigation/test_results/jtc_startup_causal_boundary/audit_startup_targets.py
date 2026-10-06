#!/usr/bin/env python3
"""Offline producer-target/independent-body comparison; never starts ROS or sends commands."""
import hashlib
import json
import math
import re
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEMO = HERE.parents[2]
RUNS = ("20261001_203152_88725b", "20261001_212305_774068")


def read_prefix(path, time_key, end=18.0):
    rows = []
    digest = hashlib.sha256()
    consumed = 0
    stop = None
    with path.open("rb") as f:
        for line_number, line in enumerate(f, 1):
            digest.update(line)
            consumed += len(line)
            row = json.loads(line)
            value = row.get(time_key)
            if value is not None and value > end:
                stop = {"line": line_number, "time": value}
                break
            rows.append(row)
    return rows, {"path": str(path), "consumed_prefix_sha256": digest.hexdigest(),
                  "consumed_bytes": consumed, "first_excluded_row": stop,
                  "whole_file_hashed": False}


def rpy(q):
    x, y, z, w = q
    n = math.sqrt(sum(v*v for v in q))
    x, y, z, w = (v/n for v in q)
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def analyze(run_id):
    run = DEMO / "runs" / run_id
    adapter, adapter_input = read_prefix(run / "joint_stop_adapter.jsonl", "sim")
    pose, pose_input = read_prefix(run / "pose_audit.jsonl", "stamp")
    nominal = next(x for x in adapter if x["kind"] == "nominal_frozen")
    targets = [x for x in adapter if x["kind"] == "filtered_target" and 10 <= x["sim"] <= 17]
    targets18 = [x for x in adapter if x["kind"] == "filtered_target" and 10 <= x["sim"] <= 18]
    commands = [x for x in adapter if x["kind"] == "actual_champ_command" and x["sim"] <= 17]
    truth = [x for x in pose if x["source"] == "truth" and 10 <= x["stamp"] <= 18]
    baseline_z = statistics.median(x["p"][2] for x in truth if x["stamp"] <= 15)
    motion = [x for x in truth if x["stamp"] > 15 and
              (abs(x["p"][2]-baseline_z) > .002 or max(abs(v) for v in rpy(x["q"])[:2]) > .01)]
    gaps = sorted(({"wall_gap_s": b["wall"]-a["wall"], "sim_delta_s": b["sim"]-a["sim"],
                    "before_sim": a["sim"], "after_sim": b["sim"],
                    "before_wall": a["wall"], "after_wall": b["wall"]}
                   for a,b in zip(targets18, targets18[1:])), key=lambda x:x["wall_gap_s"], reverse=True)
    selected = []
    for t in (10, 13, 15.8, 15.9, 16, 16.2, 16.5, 16.94, 17):
        x = min(truth, key=lambda x: abs(x["stamp"]-t))
        if abs(x["stamp"]-t) > .020001:
            raise ValueError("Requested truth example lacks an actual source sample within 20ms")
        selected.append({"requested_time_s":t,"actual_stamp_ns":x["stamp_ns"],
                         "position":x["p"],"rpy":rpy(x["q"]),
                         "purpose":"Independent physical evaluation only; not a controller input"})
    init = []
    for line_number,line in enumerate((run/"stack.log").open(),1):
        if "[DEMO_IMU_INIT]" in line:
            match = re.search(r"stable=(\d+).*?first=([\d.]+) last=([\d.]+).*?resets=(\d+) reason=(\S+)",line)
            if match and float(match[3]) <= 18:
                init.append({"line":line_number,"stable_samples":int(match[1]),
                             "first":float(match[2]),"last":float(match[3]),
                             "resets":int(match[4]),"reason":match[5]})
    return {
        "run_id":run_id,"inputs":[adapter_input,pose_input],
        "nominal_frozen":nominal,"filtered_target_count_10_to_17":len(targets),
        "target_modes_10_to_17":sorted({x["mode"] for x in targets}),
        "max_abs_nominal_target_delta":max(abs(v-n) for x in targets for v,n in zip(x["positions"],nominal["positions"])),
        "actual_champ_command_count_through_17":len(commands),
        "actual_nonzero_champ_command_count_through_17":sum(max(map(abs,x["value"]))>0 for x in commands),
        "max_target_wall_gap_10_to_17_s":max(b["wall"]-a["wall"] for a,b in zip(targets,targets[1:])),
        "largest_target_wall_gaps_10_to_18":gaps[:4],
        "truth_baseline_z_median_10_to_15":baseline_z,
        "first_physical_excursion_after_15":({"stamp_ns":motion[0]["stamp_ns"],"position":motion[0]["p"],"rpy":rpy(motion[0]["q"])} if motion else None),
        "diagnostic_excursion_definition":"After15s: absolute height change>2mm from10–15s median OR |roll/pitch|>.01rad; diagnostic only, not an acceptance threshold",
        "truth_examples":selected,
        "startup_gate":json.loads((run/"startup_gate.json").read_text()),
        "fastlivo_init_through_18":init,
        "jtc_reference_feedback_output_recorded":False,
        "limits":["Adapter filtered_target records publisher-side output after publish returns, not JTC consumption or applied torque.",
                  "Truth is independent evaluation only. Joint measured states and JTC internal periods are not available in these original two runs."]}


def main():
    result={"scope":"Read-only startup comparison; neither run nor acceptance is modified", "runs":[analyze(r) for r in RUNS]}
    result["nominal_frozen_arrays_exactly_equal"] = result["runs"][0]["nominal_frozen"]["positions"] == result["runs"][1]["nominal_frozen"]["positions"]
    keys=["simulation/config/ros_control.yaml","simulation/generated/go2.urdf","simulation/joint_reference_adapter.py","simulation/joint_stop_core.py"]
    manifests=[json.loads((DEMO/"runs"/r/"source_manifest.json").read_text())["sha256"] for r in RUNS]
    result["archived_source_configuration"]={k:{"full15":manifests[0].get(k),"full16":manifests[1].get(k),"equal":manifests[0].get(k)==manifests[1].get(k)} for k in keys}
    result["runtime_controller_identity"]={r:{k:v for k,v in json.loads((DEMO/"runs"/r/"runtime_manifest.json").read_text())["artifacts"].items() if k in ("joint_trajectory_controller","gz_ros2_control","champ_base_controller")} for r in RUNS}
    (HERE/"startup_target_comparison.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"nominal_arrays_exactly_equal":result["nominal_frozen_arrays_exactly_equal"],
                      "runs":[{k:v for k,v in r.items() if k in ("run_id","max_abs_nominal_target_delta","actual_nonzero_champ_command_count_through_17","max_target_wall_gap_10_to_17_s","first_physical_excursion_after_15")} for r in result["runs"]]},indent=2))


if __name__ == "__main__":
    main()
