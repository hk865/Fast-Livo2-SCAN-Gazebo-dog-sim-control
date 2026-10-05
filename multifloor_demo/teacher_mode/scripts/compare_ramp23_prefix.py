#!/usr/bin/env python3
"""Compare actual Gazebo ramp trials with finite Isaac CPU prefixes, read only.

Writes new comparison artifacts, never modifies original runs or trains a policy.
"""
from __future__ import annotations
import argparse
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
    spec = importlib.util.spec_from_file_location(prefix + sha(path)[:10], path)
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


def audit_isaac(prefix, name, actor):
    import torch
    rows = read(prefix / f"ramp23_{name}_prefix_trace.json")
    observation = module(ROOT / "policy/observation.py", "prefix_geometry_")
    terrain = observation.TerrainHeightMap.from_sdf(prefix / "fixture_world.sdf")
    obs = np.asarray([r["observation247"] for r in rows], dtype=np.float32)
    actions = np.asarray([r["action"] for r in rows], dtype=np.float32)
    live = np.asarray([r["controller_mode"] == "teacher" for r in rows])
    with torch.inference_mode():
        predicted = actor(torch.from_numpy(obs[live])).numpy()
    geom_error, raw_clip_error = 0., 0.
    for row in rows:
        hits = np.asarray(row["height_ray_hits_world"])
        exact = terrain.height(hits[:, :2], ray_start_z=row["scanner_position_world"][2]+20)
        geom_error = max(geom_error, float(abs(exact-hits[:, 2]).max()))
        raw_clip_error = max(raw_clip_error, float(abs(np.clip(row["height_scan_raw"], -1, 1)-np.asarray(row["height_scan"])).max()))
    t = np.asarray([r["t"] for r in rows])
    pose = np.asarray([r["position"] for r in rows])
    signs = -1 if name == "up" else 1
    active = t >= 3
    progress = signs*(pose[:, 0]-pose[0, 0])
    init = (t >= .1) & (t < 3)
    mean_world_progress = signs*(pose[-1, 0]-np.median(pose[init, 0]))
    cf = np.asarray([r["base_contact_force_world"] for r in rows])
    result = next(r for r in read(prefix / "results.json")["results"] if r["name"] == f"ramp23_{name}_prefix")
    report = {
        "run": str(prefix), "name": f"ramp23_{name}_prefix", "planned_prefix_s": 30,
        "prestep_last_snapshot_s": t[-1], "actual_advanced_duration_s": result["duration_recorded"],
        "native_or_manual_failure": result["failed"], "raw_script_passed_flag": result["passed"],
        "raw_flag_scope": "Only finite prefix numeric diagnostic; no complete ramp or final stop was scheduled",
        "forward_progress_world_x_from_spawn_m": progress[-1], "progress_from_precommand_median_m": mean_world_progress,
        "final_position": pose[-1], "maximum_lateral_drift_from_y7_m": abs(pose[active, 1]-7).max(),
        "minimum_physical_clearance_since_teacher_s": min(r["clearance"] for r in rows if r["t"] >= .1),
        "maximum_abs_roll_pitch_since_teacher_rad": max(max(abs(r["roll"]), abs(r["pitch"])) for r in rows if r["t"] >= .1),
        "prestep_base_net_contact_force_positive_samples": int((np.linalg.norm(cf.reshape(len(rows), -1), axis=1) > 1).sum()),
        "cpu_actor_replay_max_error": float(abs(predicted-actions[live]).max()),
        "actual_observed_command_vs_requested_effective_command_max_error": float(abs(obs[:, 9:12]-np.asarray([r["command"] for r in rows])).max()),
        "actual_raw_scan_clip_vs_actor_scan_max_error": raw_clip_error,
        "actual_mesh_hit_xyz_vs_exactSDF_at_recorded_hitXY_max_error_m": geom_error,
        "termination_limits": "The script logs pre-step snapshots, then checks env.step terminated. It did not save final post-step state or individual native termination flags. Failed up cause cannot be uniquely reconstructed.",
        "initial_snapshot_cache": {"t0_logged_COM_velocity": rows[0]["linear_velocity_body_com"], "t0_logged_angular_velocity": rows[0]["angular_velocity_body"],
                                   "t0_excluded_from_teacher": rows[0]["controller_mode"] == "pd_init",
                                   "t002_velocity": rows[1]["linear_velocity_body_com"],
                                   "interpretation": "Reset-time derived buffers retain previous step values at t0; PD action is explicitly zero/default target and state refreshes after physics. Do not interpret initial logged twist as a requested nonzero reset velocity."},
        "source_hashes": {f: sha(prefix / f) for f in [f"ramp23_{name}_prefix_trace.json", "results.json", "protocol.json", "fixture_world.sdf", "fixture_manifest.json"]},
    }
    return report, {"t": t, "progress": progress, "lateral": pose[:, 1]-7, "clearance": np.asarray([r["clearance"] for r in rows])}


def audit_gazebo(run, name):
    rows = lines(run / "telemetry.jsonl")
    full = read(run / "summary_full_ramp.json")
    t = np.asarray([r["sim_time"] for r in rows])
    pose = np.asarray([r["position"] for r in rows])
    sign = -1 if name == "up" else 1
    report = {"run": str(run), "actual_full_ramp_status": full["status"], "failed_checks": [k for k, v in full["checks"].items() if not v["passed"]],
              "actual_recorded_duration_s": t[-1], "metrics": full["metrics"],
              "first_worker_fault": next(({"time_s": r["sim_time"], "reason": r["fault"]} for r in rows if r.get("fault")), None),
              "source_hashes": {f: sha(run / f) for f in ["telemetry.jsonl", "actuator.jsonl", "summary_full_ramp.json", "world.sdf", "policy_manifest.json", "asset_manifest.json", "independent_protocol.json"]}}
    return report, {"t": t, "progress": sign*(pose[:, 0]-pose[0, 0]), "lateral": pose[:, 1]-7,
                    "clearance": np.asarray([r["body_clearance"] for r in rows])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--gazebo-up", type=Path, required=True)
    parser.add_argument("--gazebo-down", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="A new directory")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    helper = module(ROOT / "scripts/analyze_terrain_target_ab.py", "prefix_actor_")
    checkpoint = Path(read(args.prefix / "protocol.json")["checkpoint"])
    actor = helper.actor(checkpoint)
    reports, plotdata = [], []
    for name, gzrun in [("up", args.gazebo_up), ("down", args.gazebo_down)]:
        isaac, idata = audit_isaac(args.prefix.resolve(), name, actor)
        gazebo, gdata = audit_gazebo(gzrun.resolve(), name)
        reports.append({"direction": name, "Isaac_CPU": isaac, "Gazebo": gazebo})
        plotdata.append((name, idata, gdata))
    protocol = read(args.prefix / "protocol.json")
    report = clean({"schema_version": 1, "status": "comparison_complete", "scope": "Actual matched ramp23 command prefixes; no full-ramp or stopping pass inferred from a30s prefix",
                    "cases": reports, "common": {"frozen_checkpoint_sha256": protocol["checkpoint_sha256"], "physics_dt_s": .005,
                        "decimation": 4, "policy_hz": 50, "command": ".1s default PD, teacher0 until3, bodyvx+.3 with slew.6/.6/.8 after3", "noise": "disabled"},
                    "material_dynamics_limits": {"Isaac_nominal_robot_mass_kg": 16.087, "Gazebo_robot_mass_kg": 16.512,
                        "Isaac_robot_and_terrain_material_coefficients": [1, 1], "Gazebo_robot_and_terrain_coefficients": [.7, 1],
                        "solver": "PhysX CPU versus Gazebo physics backend", "geometry": "Same complete static SDF mesh in Isaac; training robot and Gazebo robot inertial/collision properties differ",
                        "termination": "Isaac scan-based height native termination disabled for this diagnostic; additional native terminations remain, final post-step reason was not logged. Gazebo full protocol safety has separate thresholds/contact checks",
                        "phase": "Gazebo firstTeacher effectivephysicaltime≈.105s, Isaac .1s; initial5ms freefall and engine/cache phases differ"},
                    "conclusion": "Up prefixes fail in both nominal Isaac and Gazebo under common commanded motion; this supports a limitation beyond mere model loading, but different physics/robot and termination criteria prevent unique cause attribution. Down survives Isaac30s but Gazebo fails25.48s; this differential motivates dynamics/adapter diagnostics and is not complete ramp success.",
                    "remaining_unverified": ["Complete12m ramp plus both landing transitions and final stop", "Strict equal dynamics or a unique policy-versus-adapter root cause", "SLAM navigation", "Real hardware"],
                    "analyzer_sha256": sha(__file__), "prefix_execution_receipt": str(ROOT / "test_results/full_ramp_isaac_prefix_20261004/execution_receipt.json"),
                    "prefix_execution_receipt_sha256": sha(ROOT / "test_results/full_ramp_isaac_prefix_20261004/execution_receipt.json")})
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), constrained_layout=True)
    for row, (name, isaac, gazebo) in enumerate(plotdata):
        for column, (field, title) in enumerate([("progress", "World forward progress (m)"), ("lateral", "Lateral offset from ramp center (m)"), ("clearance", "Body support clearance (m)")]):
            ax = axes[row, column]
            ax.plot(isaac["t"], isaac[field], label="Isaac CPU prefix", color="#2563eb", lw=1.2)
            ax.plot(gazebo["t"], gazebo[field], label="Gazebo actual", color="#dc2626", lw=1.2)
            ax.axvline(3, color="gray", lw=.7, ls=":")
            if field == "clearance":
                ax.axhline(.18, color="gray", lw=.8, ls="--", label="Gazebo full-protocol 0.18m")
                ax.axhline(.16, color="#6b7280", lw=.6, ls=":", label="Isaac diagnostic 0.16m")
            ax.set_title(name.upper()+" · "+title)
            ax.set_xlim(0, 30)
            ax.grid(alpha=.2)
            ax.set_xlabel("Logical test time (s)")
            ax.legend(fontsize=7)
    fig.suptitle("Frozen Teacher: actual ramp23 motion prefixes, different nominal dynamics\nFinite prefix survival does not prove full traversal or stopping", fontsize=12)
    fig.savefig(args.output / "ramp23_prefix_comparison.png", dpi=150)
    plt.close(fig)
    print(json.dumps({"report": str(args.output / "comparison.json"), "figure": str(args.output / "ramp23_prefix_comparison.png"), "Isaac": [{"direction": r["direction"], "failed": r["Isaac_CPU"]["native_or_manual_failure"], "advanced_s": r["Isaac_CPU"]["actual_advanced_duration_s"], "world_progress_m": r["Isaac_CPU"]["forward_progress_world_x_from_spawn_m"]} for r in reports]}))


if __name__ == "__main__":
    main()
