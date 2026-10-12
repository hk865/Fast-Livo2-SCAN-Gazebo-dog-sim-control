#!/usr/bin/env python3
"""Read immutable terrain-target A/B logs; write a NEW audit receipt only.

No ROS node, simulator, process signal, policy switch or navigation operation.
Offline CPU forwards are counterfactual sensitivity diagnostics, not execution.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
SHA = "bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def lines(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def observation_module(run):
    path = run / "sources/policy/observation.py"
    spec = importlib.util.spec_from_file_location("ab_obs_" + sha(path)[:12], path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def actor(checkpoint):
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    if sha(checkpoint) != SHA:
        raise ValueError("Frozen checkpoint mismatch")
    model = torch.nn.Sequential(
        torch.nn.Linear(247, 512), torch.nn.ELU(), torch.nn.Linear(512, 256), torch.nn.ELU(),
        torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 12),
    ).eval()
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)["actor_state_dict"]
    model.load_state_dict({k.removeprefix("mlp."): v for k, v in state.items() if k.startswith("mlp.")}, strict=True)
    return model


def load_run(run, model):
    import torch
    run = run.resolve()
    meta = read(run / "policy_manifest.json")
    rows = lines(run / "telemetry.jsonl")
    scans = lines(run / "height_scan_sources.jsonl")
    native = lines(run / "actuator.jsonl")
    steps = [row for row in native if row.get("kind") == "physics_step"]
    contract = next(row for row in native if row.get("kind") == "actuator_contract")
    asset = read(run / "asset_manifest.json")
    summary = read(run / "summary.json")
    result = read(run / "worker_result.json")
    snapshot = read(run / "source_manifest.json")
    with np.load(run / "observations_actions.npz") as saved:
        obs = saved["observations"].copy()
        actions = saved["actions"].copy()
    if obs.shape != (len(rows), 247) or actions.shape != (len(rows), 12) or len(scans) != len(rows):
        raise ValueError(f"Incomplete archived inputs: {run}")
    module = observation_module(run)
    terrain = module.TerrainHeightMap.from_sdf(run / "world.sdf", meta["terrain_target_manifest"]["definition"]["include_models"])
    safety = module.TerrainHeightMap.from_sdf(run / "world.sdf")
    rebuilt, name_errors, height_clip_errors = [], 0, []
    last_action = np.zeros(12)
    for row, scan, actual in zip(rows, scans, obs):
        state = np.zeros(64)
        state[0] = row["world_sim_time"]
        state[1:4] = row["position"]
        state[4:8] = row["quaternion_wxyz"]
        state[8:11] = row["body_lin_vel"]
        state[11:14] = row["body_ang_vel"]
        state[14:26] = row["q"]
        state[26:38] = row["qd"]
        state[38:50] = row["applied_torque"]
        candidate, extra = module.build_observation(state, row["command"], last_action, terrain)
        rebuilt.append(candidate)
        name_errors += int(extra["height_scan_hit_models"] != scan["height_scan_hit_models"])
        raw = np.asarray([float(v) if v is not None else -np.inf for v in scan["height_scan_raw"]])
        height_clip_errors.append(float(np.max(abs(np.clip(raw, -1, 1) - actual[60:]))))
        last_action = np.asarray(row["action"])
    t = np.asarray([row["sim_time"] for row in rows])
    live = np.asarray([row["sim_time"] >= meta["bootstrap_PD_seconds"] - 1e-9 and not row.get("fault") for row in rows])
    with torch.inference_mode():
        predicted = model(torch.from_numpy(obs[live].astype(np.float32))).numpy()
    offset = rows[0]["world_sim_time"] - rows[0]["sim_time"]
    nt = np.asarray([row["t"] - offset for row in steps])
    pos = np.asarray([row["position"] for row in steps])
    ground = safety.height(pos[:, :2], pos[:, 2])
    clearance = pos[:, 2] - ground
    valid = nt >= meta["bootstrap_PD_seconds"] - 1e-9
    angles = []
    for row in steps:
        r = module.quaternion_rotation(row["quaternion_wxyz"])
        angles.append([np.arctan2(r[2, 1], r[2, 2]), np.arcsin(np.clip(-r[2, 0], -1, 1)), np.arctan2(r[1, 0], r[0, 0])])
    angles = np.asarray(angles)
    contacts = np.asarray([row["contacts"] for row in steps])
    first = scans[0]
    motion_metrics = summary["tests"][0]["metrics"]
    files = ["world.sdf", "asset_manifest.json", "policy_manifest.json", "terrain_target_manifest.json",
             "telemetry.jsonl", "actuator.jsonl", "height_scan_sources.jsonl", "observations_actions.npz",
             "worker_result.json", "summary.json", "runtime_manifest.json", "source_manifest.json",
             "sources/policy/worker.py", "sources/policy/observation.py", "sources/policy/contract.json",
             "sources/simulation/prepare.py", "sources/simulation/teacher_actuator.cpp", "sources/tests/protocol.json"]
    audit = {
        "run": str(run), "actual_trajectory": True, "actor_device": meta["inference_device"],
        "checkpoint_sha256": meta["checkpoint_sha256"], "target": meta["terrain_target_manifest"],
        "duration_s": t[-1], "scheduled_duration_s": meta["test_duration_s"], "policy_rows": len(rows),
        "worker_fault": result.get("fault"), "levels_from_immutable_summary": summary["levels"],
        "all_actual_command_zero": bool(np.max(abs(np.asarray([row["command"] for row in rows]))) == 0),
        "first_scan": {"ray_hit_models": dict(Counter(n.split("/")[0] for n in first["height_scan_hit_models"] if n)),
                       "overhead_count": first["height_scan_overhead_count"], "valid_count": first["height_scan_valid_count"],
                       "raw_range": [first["height_scan_raw_min"], first["height_scan_raw_max"]],
                       "policy_range": [obs[0, 60:].min(), obs[0, 60:].max()]},
        "all_scan": {"overhead_frames": sum(s["height_scan_overhead_count"] > 0 for s in scans),
                     "overhead_ray_total": sum(s["height_scan_overhead_count"] for s in scans),
                     "unknown_ray_total": sum(s["height_scan_invalid_count"] for s in scans),
                     "reconstructed_hit_name_mismatch_frames": name_errors,
                     "raw_clip_vs_actual247_max_error": max(height_clip_errors)},
        "independent_replay": {"247_from_logged_state_history_max_error": float(np.max(abs(np.asarray(rebuilt) - obs))),
                               "actual_recorded247_to_cpuactor_max_action_error": float(np.max(abs(predicted - actions[live]))),
                               "teacher_rows": int(live.sum()), "pd_and_fault_rows_excluded_from_actor_replay": int((~live).sum())},
        "native_200hz_safety_since_bootstrap": {
            "samples": int(valid.sum()), "min_all_collision_support_clearance_m": float(clearance[valid].min()),
            "max_abs_roll_pitch_rad": float(abs(angles[valid, :2]).max()),
            "body_contact_samples": int((contacts[valid, 0] > 0).sum()),
            "unknown_contact_samples": int((contacts[valid] < 0).sum()),
            "max_tau_Nm": float(abs(np.asarray([s["tau"] for s in steps])[valid]).max()),
        },
        "stand_metrics_from_frozen_protocol": motion_metrics.get("stop_teacher"),
        "source_hashes": {rel: sha(run / rel) for rel in files},
        "archived_source_hash_mismatches": [rel for rel, h in snapshot.items() if not (run / "sources" / rel).exists() or sha(run / "sources" / rel) != h],
    }
    return {"audit": audit, "rows": rows, "obs": obs, "actions": actions, "t": t, "asset": asset,
            "meta": meta, "contract": contract, "snapshot": snapshot, "live": live, "scans": scans}


def evaluate(a_path, b_path):
    import torch
    checkpoint = Path(read(a_path / "policy_manifest.json")["checkpoint"])
    model = actor(checkpoint)
    a, b = load_run(a_path, model), load_run(b_path, model)
    core = ["policy/worker.py", "policy/observation.py", "policy/contract.json", "simulation/prepare.py",
            "simulation/teacher_actuator.cpp", "scripts/run_test.py", "scripts/evaluate.py", "tests/protocol.json"]
    keys = ["spawn", "physics_step_s", "decimation", "default_q", "hard_lower", "hard_upper", "friction_baseline",
            "model_mass_kg", "base_inertial_pose", "world_sha256"]
    controls = ["inference_device", "observation_noise", "seed", "stop_strategy", "bootstrap_PD_seconds",
                "command_slew_acceleration", "test_duration_s", "checkpoint_sha256"]
    common = {"same_exact_world_bytes": sha(a_path / "world.sdf") == sha(b_path / "world.sdf"),
              "same_asset_physics_fields": all(a["asset"].get(k) == b["asset"].get(k) for k in keys),
              "same_native_actuator_contract": a["contract"] == b["contract"],
              "same_core_source_hashes": all(a["snapshot"].get(k) == b["snapshot"].get(k) for k in core),
              "same_policy_control_fields": all(a["meta"].get(k) == b["meta"].get(k) for k in controls),
              "both_cpu_frozen_actor": a["meta"]["inference_device"] == b["meta"]["inference_device"] == "cpu" and a["meta"]["checkpoint_sha256"] == b["meta"]["checkpoint_sha256"] == SHA,
              "both_actual_cmd_zero": a["audit"]["all_actual_command_zero"] and b["audit"]["all_actual_command_zero"]}
    n = min(len(a["t"]), len(b["t"]))
    if not np.allclose(a["t"][:n], b["t"][:n], atol=1e-10):
        raise ValueError("A/B policy clock phases differ")
    init = a["t"][:n] < a["meta"]["bootstrap_PD_seconds"] - 1e-9
    k = int(np.flatnonzero(a["live"][:n] & b["live"][:n])[0])
    def arr(d, key):
        return np.asarray([row[key] for row in d["rows"][:n]])
    bootstrap = {key: float(np.max(abs(arr(a, key)[init] - arr(b, key)[init])))
                 for key in ["position", "quaternion_wxyz", "q", "qd", "q_target", "applied_torque"]}
    with torch.inference_mode():
        av = model(torch.from_numpy(a["obs"][k:k+1].astype(np.float32))).numpy()[0]
        bv = model(torch.from_numpy(b["obs"][k:k+1].astype(np.float32))).numpy()[0]
        swapped = a["obs"][k:k+1].copy()
        swapped[:, 60:] = b["obs"][k, 60:]
        cf = model(torch.from_numpy(swapped.astype(np.float32))).numpy()[0]
    difference = {
        "first_teacher_time_s": float(a["t"][k]),
        "first_teacher_nonheight_0_60_max_error": float(np.max(abs(a["obs"][k, :60] - b["obs"][k, :60]))),
        "first_teacher_height_60_247_max_error": float(np.max(abs(a["obs"][k, 60:] - b["obs"][k, 60:]))),
        "actual_first_teacher_action_A": a["actions"][k], "actual_first_teacher_action_B": b["actions"][k],
        "actual_action_difference_max": float(abs(a["actions"][k] - b["actions"][k]).max()),
        "actual_qtarget_difference_max_rad": float(abs(arr(a, "q_target")[k] - arr(b, "q_target")[k]).max()),
        "actor_replay_first_A_max_error": float(abs(av - a["actions"][k]).max()),
        "actor_replay_first_B_max_error": float(abs(bv - b["actions"][k]).max()),
        "counterfactual_A_state_only_B_height_action": cf,
        "counterfactual_vs_actual_first_B_max_error": float(abs(cf - b["actions"][k]).max()),
        "counterfactual_scope": "Offline CPU sensitivity only. It is not an executed replacement trajectory; later trajectories have different states/history.",
    }
    replay = all(r["audit"]["independent_replay"]["247_from_logged_state_history_max_error"] <= 2e-5 and
                 r["audit"]["independent_replay"]["actual_recorded247_to_cpuactor_max_action_error"] <= 2e-5 and
                 not r["audit"]["archived_source_hash_mismatches"] for r in [a, b])
    return clean({"schema_version": 1, "audit_status": "complete" if all(common.values()) and replay else "audit_checks_failed",
                  "scope": "One paired actual origin zero-command Gazebo A/B using privileged fixed terrain target selection; all physical collisions and safety unchanged.",
                  "A": a["audit"], "B": b["audit"], "common_conditions": common,
                  "bootstrap_before_first_teacher_max_errors": bootstrap, "first_teacher_input_action_effect": difference,
                  "source_note": "Physical outcomes are immutable source summaries. A failed motion is preserved even when the audit itself completes.",
                  "limitations": ["Only one paired repetition", "Fixed privileged deployment target sets are not strict frozen training geometry equivalence",
                                  "No sensor-derived input replacement", "No SLAM navigation success", "No real robot validation", "No claim of unique global failure cause"],
                  "analysis_source": str(Path(__file__).resolve()), "analysis_sha256": sha(__file__)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--a", type=Path, required=True, help="A all-static run")
    parser.add_argument("--b", type=Path, required=True, help="B fixed lower terrain run")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Preserve old receipts: output must be a new file")
    receipt = evaluate(args.a.resolve(), args.b.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"audit_status": receipt["audit_status"], "output": str(args.output),
                      "A_actual_motion": receipt["A"]["levels_from_immutable_summary"]["motion"],
                      "B_actual_motion": receipt["B"]["levels_from_immutable_summary"]["motion"],
                      "common_conditions": receipt["common_conditions"],
                      "first_teacher_action_difference_max": receipt["first_teacher_input_action_effect"]["actual_action_difference_max"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
