#!/usr/bin/env python3
"""Read-only audit of actual ROS input replacement; write new independent receipts.

Raw simulation logs are never changed. Imports no ROS and starts no simulator.
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


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def lines(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def module(path, prefix):
    spec = importlib.util.spec_from_file_location(prefix + sha(path)[:12], path)
    result = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = result
    spec.loader.exec_module(result)
    return result


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


def statistics(values):
    values = np.asarray(values, dtype=float)
    return None if not len(values) else {"min": values.min(), "p50": np.quantile(values, .5), "p95": np.quantile(values, .95), "max": values.max()}


def audit(run, actor):
    import torch
    run = Path(run).resolve()
    rows, samples, native = lines(run / "telemetry.jsonl"), lines(run / "sensor_feedback_samples.jsonl"), lines(run / "actuator.jsonl")
    meta, feedback, summary = read(run / "policy_manifest.json"), read(run / "sensor_feedback.json"), read(run / "summary.json")
    config = read(run / "sources/policy/contract.json")
    observation = module(run / "sources/policy/observation.py", "sensor_audit_obs_")
    terrain = observation.TerrainHeightMap.from_sdf(run / "world.sdf", meta.get("terrain_target_manifest", {}).get("definition", {}).get("include_models"))
    default_q = np.asarray(config["default_joint_positions"], dtype=float)
    with np.load(run / "observations_actions.npz") as saved:
        obs, actions = saved["observations"].copy(), saved["actions"].copy()
    if obs.shape != (len(rows), 247) or actions.shape != (len(rows), 12):
        raise ValueError("Actual247/action logs do not cover every policy row")
    broker = module(run / "sources/scripts/sensor_feedback.py", "sensor_audit_broker_")
    broker_contract = broker.FeedbackContract(run)
    accepted = {key: [s for s in samples if s["stream"] == key] for key in ("imu", "joints")}
    indexed = {key: {s["stamp_ns"]: s for s in stream} for key, stream in accepted.items()}
    if any(len(indexed[key]) != len(stream) for key, stream in accepted.items()):
        raise ValueError("Accepted sensor logs contain duplicate stamps")
    valid = np.isfinite(obs).all(axis=1)
    inferred = np.asarray([bool(row.get("actor_inferred_this_frame")) for row in rows])
    last = np.zeros(12)
    replaced_indices = np.r_[3:9, 12:36]
    retained_indices = np.setdiff1d(np.arange(247), replaced_indices)
    reconstruction_error = retained_error = raw_error = frame_error = mapping_error = 0.
    first_replacement = None
    used = []
    checks = {"cpu_frozen_actor": meta.get("inference_device") == "cpu" and meta.get("checkpoint_sha256") == "bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34",
              "exactly_30_named_dimensions": True, "activation_at_t3": True, "causal_no_future_source": True,
              "sim_age_at_most_25ms": True, "wall_age_at_most_300ms": True, "actual_callback_source_exists": True,
              "no_truth_fallback": True, "broker_hash_matches_archive": True}
    ages, wallages, export_lags = {k: [] for k in accepted}, {k: [] for k in accepted}, {k: [] for k in accepted}
    received = {k: np.asarray([s["received_monotonic_wall"] for s in stream]) for k, stream in accepted.items()}
    stamps = {k: np.asarray([s["stamp_ns"] for s in stream], dtype=np.int64) for k, stream in accepted.items()}
    for index, row in enumerate(rows):
        state = np.zeros(64)
        state[0] = row["world_sim_time"]
        for sl, field in [(slice(1, 4), "position"), (slice(4, 8), "quaternion_wxyz"), (slice(8, 11), "body_lin_vel"),
                          (slice(11, 14), "body_ang_vel"), (slice(14, 26), "q"), (slice(26, 38), "qd"), (slice(38, 50), "applied_torque")]:
            state[sl] = row[field]
        baseline, _ = observation.build_observation(state, row["command"], last, terrain)
        expected = baseline.copy()
        e = row.get("sensor_feedback_used")
        if row["sim_time"] < 3 and e is not None:
            checks["activation_at_t3"] = False
        if row["sim_time"] >= 3 and inferred[index] and e is None:
            checks["activation_at_t3"] = False
        if e:
            used.append(index)
            first_replacement = row["sim_time"] if first_replacement is None else first_replacement
            effective = round((row["world_sim_time"] - .005)*1e9)
            checks["causal_no_future_source"] &= effective == e["effective_physics_stamp_ns"]
            checks["exactly_30_named_dimensions"] &= e["dimension_count"] == 30 and e["replaced_dimensions"] == [3, 6, 12, 24]
            checks["no_truth_fallback"] &= e["truth_fallback"] is False and e["actor_input_changed"] is True
            checks["broker_hash_matches_archive"] &= e["broker_sha256"] == sha(run / "sources/scripts/sensor_feedback.py")
            raw = e["raw_terms"]
            for key in accepted:
                source = e["sources"][key]
                sample = indexed[key].get(source["stamp_ns"])
                if sample is None:
                    checks["actual_callback_source_exists"] = False
                    continue
                checks["causal_no_future_source"] &= 0 <= effective-source["stamp_ns"]
                checks["sim_age_at_most_25ms"] &= 0 <= source["age_sim_s"] <= .025+1e-12
                checks["wall_age_at_most_300ms"] &= 0 <= source["age_wall_s"] <= .3
                checks["actual_callback_source_exists"] &= sample["source"] == "actual_ros_subscription_callback" and sample["source_topic"] == source["source_topic"] == broker.TOPICS[key] and sample["frame"] == source["frame"]
                ages[key].append(source["age_sim_s"])
                wallages[key].append(source["age_wall_s"])
                read_wall = sample["received_monotonic_wall"] + source["age_wall_s"]
                causal = (received[key] <= read_wall) & (stamps[key] <= effective)
                newest = stamps[key][causal].max() if causal.any() else source["stamp_ns"]
                export_lags[key].append((newest-source["stamp_ns"])*1e-9)
                for field in (["gyro_body", "gravity_body"] if key == "imu" else ["q", "qd"]):
                    raw_error = max(raw_error, float(abs(np.asarray(sample[field])-np.asarray(raw[field])).max()))
                if key == "imu":
                    independent_gyro = broker_contract.body_imu @ np.asarray(sample["gyro_imu"])
                    world_body = broker_contract.world_reference @ broker.rotation(sample["orientation_xyzw"]) @ broker_contract.body_imu.T
                    independent_gravity = world_body.T @ np.asarray([0., 0., -1.])
                    frame_error = max(frame_error, float(abs(independent_gyro-np.asarray(raw["gyro_body"])).max()), float(abs(independent_gravity-np.asarray(raw["gravity_body"])).max()))
                else:
                    order = [sample["message_names"].index(n) for n in config["gazebo_joint_names"]]
                    mapping_error = max(mapping_error, float(abs(np.asarray(sample["message_position"])[order]-np.asarray(raw["q"])).max()), float(abs(np.asarray(sample["message_velocity"])[order]-np.asarray(raw["qd"])).max()))
            expected[3:6] = np.clip(raw["gyro_body"], -100, 100)*.2
            expected[6:9] = np.clip(raw["gravity_body"], -100, 100)
            expected[12:24] = np.clip(np.asarray(raw["q"])-default_q, -100, 100)
            expected[24:36] = np.clip(raw["qd"], -100, 100)*.05
        if valid[index]:
            reconstruction_error = max(reconstruction_error, float(abs(expected-obs[index]).max()))
            retained_error = max(retained_error, float(abs(baseline[retained_indices]-obs[index, retained_indices]).max()))
        last = np.zeros(12) if row["state"] == "initializing" else np.asarray(row["action"])
    with torch.inference_mode():
        predicted = actor(torch.from_numpy(obs[valid & inferred].astype(np.float32))).numpy()
    actor_error = float(abs(predicted-actions[valid & inferred]).max())
    checks.update(actual247_reconstructed=reconstruction_error <= 2e-5, actual_sensor_terms_equal_callback_log=raw_error == 0,
                  frame_rotation_verified=frame_error <= 1e-12, named_q_qd_mapping_verified=mapping_error == 0,
                  nonreplaced_217_dimensions_retained=retained_error == 0, cpu_actor_replay=actor_error <= 2e-5)
    steps = [r for r in native if r.get("kind") == "physics_step"]
    faults = [r for r in native if r.get("kind") == "fault"]
    rejected = [r for r in rows if r.get("fault")]
    rejection = None
    if rejected:
        first = rejected[0]
        terminal = steps[-1]
        rejection = {"time_s": first["sim_time"], "reason": first["fault"], "actor_inferred_on_rejected_frame": first.get("actor_inferred_this_frame"),
                     "sensor_evidence_present": first.get("sensor_feedback_used") is not None,
                     "stored247_finite_on_rejected_frame": bool(valid[rows.index(first)]),
                     "native_terminal_mode": terminal["mode"], "native_terminating": terminal["terminating"],
                     "native_latched_fault_events": faults,
                     "scope": "Rejected actual input stops inference and requests native damping; held logged raw action is not executed as privileged PD fallback"}
        checks["rejection_damps_without_truth_inference"] = first.get("actor_inferred_this_frame") is False and not valid[rows.index(first)] and terminal["mode"] == 1 and terminal["terminating"] is True
    files = ["telemetry.jsonl", "observations_actions.npz", "actuator.jsonl", "policy_manifest.json", "sensor_contract.json", "asset_manifest.json",
             "sensor_feedback.json", "sensor_feedback_samples.jsonl", "sensor_feedback_events.jsonl", "summary.json", "runtime_manifest.json",
             "sources/scripts/sensor_feedback.py", "sources/policy/worker.py", "sources/policy/observation.py", "sources/policy/contract.json"]
    return clean({"run": str(run), "audit_status": "source_and_replay_verified" if all(checks.values()) else "audit_failed",
                  "checks": checks, "actual_motion_levels_unchanged": summary["levels"], "actual_motion_metrics": summary["tests"],
                  "total_policy_rows": len(rows), "actual_sensor_replaced_rows": len(used), "first_replacement_s": first_replacement,
                  "actual_sensor_replaced_intervals": [[3, 6], [6, 9], [12, 24], [24, 36]],
                  "retained_217_note": "190 privileged dimensions = COM velocity3 + height187; remaining27 are controller command3, applied actuator command12 and last raw action12",
                  "maximum_observation_reconstruction_error": reconstruction_error, "maximum_nonreplaced_error": retained_error,
                  "maximum_selected_terms_vs_raw_callback_error": raw_error, "maximum_frame_rotation_error": frame_error,
                  "maximum_named_joint_mapping_error": mapping_error, "maximum_cpu_actor_action_replay_error": actor_error,
                  "source_age_sim_s": {k: statistics(v) for k, v in ages.items()}, "source_age_wall_s": {k: statistics(v) for k, v in wallages.items()},
                  "avoidable_export_lag_vs_received_causal_callback_s": {k: {"frames_with_newer_actual_received_sample": sum(x > 0 for x in v), "lag": statistics(v)} for k, v in export_lags.items()},
                  "broker_final_status": {"alive": feedback["broker_alive"], "sources": feedback["source_status"], "invalid_reasons": feedback["invalid_reason_counts"]},
                  "source_rejection": rejection, "source_hashes": {f: sha(run / f) for f in files},
                  "limits": ["Only actual Gazebo IMU/joint substitution; no hardware attitude estimator validation", "SLAM/cloud/COM velocity/height replacement not tested",
                             "Forward source rejection is an actual failed full-motion trial, even when source audit and fail-safe verify", "No unique failure cause is inferred without fault-time candidate snapshot"]})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Use a new audit receipt; never overwrite historical evidence")
    helper = module(ROOT / "scripts/analyze_terrain_target_ab.py", "sensor_actor_")
    actor = helper.actor(Path(read(args.runs[0] / "policy_manifest.json")["checkpoint"]))
    result = {"schema_version": 1, "scope": "Actual sensor replacement provenance/247/CPU actor audit; simulation only",
              "runs": [audit(run, actor) for run in args.runs], "analyzer_sha256": sha(__file__), "analyzer": str(Path(__file__).resolve())}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "runs": [{"run": r["run"], "audit_status": r["audit_status"], "actual_motion": r["actual_motion_levels_unchanged"]["motion"], "replaced_rows": r["actual_sensor_replaced_rows"], "failed_audit_checks": [k for k, v in r["checks"].items() if not v]} for r in result["runs"]]}))


if __name__ == "__main__":
    main()
