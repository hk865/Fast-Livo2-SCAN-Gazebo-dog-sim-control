"""Read original owned-clean ALIGN first8 failure windows; no ROS/CDR/control."""
import bisect
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("motion", HERE / "audit_motion.py")
motion = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion)


def audit(run, output):
    assert not output.exists()
    assert json.loads((run / "process_cleanup.json").read_text())["owned_group_clean"]
    data = json.loads((run / "first_eight_result.json").read_text())
    guard = json.loads((run / "nav_drift_guard.json").read_text())
    commands, edges = [], []
    with (run / "joint_stop_adapter.jsonl").open() as stream:
        for seq, line in enumerate(stream):
            row = json.loads(line)
            stamp = round(row["sim"] * 1e9)
            if row["kind"] == "actual_champ_command":
                commands.append((stamp, row, seq + 1))
            if 323e9 <= stamp <= 339e9 and row["kind"] in (
                "zero_edge", "native_zero_ack", "return_complete"
            ):
                row["native_file_line"] = seq + 1
                edges.append(row)
    times = [row[0] for row in commands]
    status_times = [row["received_stamp_ns"] for row in data["statuses"]]

    def status_short(row):
        status = row["status"]
        align = status.get("turn_drift_guard", {}).get("align_translation", {})
        steering = status.get("steering") or {}
        return dict(received_stamp_ns=row["received_stamp_ns"],
            phase=status.get("alignment_phase"), state=status.get("state"),
            command=status.get("command"), accepted_traj_id=status.get("accepted_trajectory_id"),
            raw_odom_stamp=steering.get("odom_stamp"), heading=steering.get("heading"),
            heading_error=steering.get("error"), locked_heading=status.get("locked_heading"),
            helper=align.get("latest"), corridor=align.get("motion_guard"),
            emit=align.get("emit"), tilt_hold=status.get("tilt_hold"))

    raw = [row for row in data["raw_imu"]
        if data["origin_stamp_ns"] <= row["stamp_ns"] <= data["terminal_stamp_ns"]]
    events = [(str(threshold), next(row for row in raw if row["tilt"] >= threshold))
        for threshold in (.2, .25, .3, .35, .5)]
    holds = 0
    for row in data["aggregate_execution_safety_events"]:
        if not data["origin_stamp_ns"] <= row["received_stamp_ns"] <= data["terminal_stamp_ns"]:
            continue
        if row["safety"].get("holds", 0) > holds:
            holds = row["safety"]["holds"]
            events.append((f"hold_{holds}", row))
    event_evidence = []
    for name, row in events:
        stamp = row.get("stamp_ns", row.get("received_stamp_ns"))
        ci = bisect.bisect_right(times, stamp) - 1
        si = bisect.bisect_right(status_times, stamp) - 1
        zero = next((c for c in commands[ci:] if c[0] >= stamp and not any(c[1]["value"])), None)
        event_evidence.append(dict(name=name, raw_or_bridge_record=row,
            native_at_event=commands[ci][1], native_at_file_line=commands[ci][2],
            native_next_zero=None if zero is None else zero[1],
            native_next_zero_file_line=None if zero is None else zero[2],
            phase_before=status_short(data["statuses"][si]),
            phase_after=status_short(data["statuses"][min(si + 1, len(status_times) - 1)])))

    common_start = max(min(row["stamp_ns"] for row in data[key]) for key in ("poses", "truth"))
    common_end = min(max(row["stamp_ns"] for row in data[key]) for key in ("poses", "truth"))
    windows = [
        ("first_020_DRIVE_bracket", 323583000000, 323883000000),
        ("first_hold_ALIGN_from_actual_arm", 325728000000, 326775000000),
        ("last_ALIGN_before_second_hold", 336800000000, 337683000000),
        ("second_hold_zero_to_terminal", 337700000000, 338414000000),
    ]
    metrics = []
    for name, original_left, original_right in windows:
        left, right = max(original_left, common_start), min(original_right, common_end)
        assert left < right
        points = sorted({left, right, *[t for t in times if left < t < right]})
        intervals = [(a, b, commands[bisect.bisect_right(times, a) - 1][1]["value"])
            for a, b in zip(points, points[1:])]
        sample_times = np.array([t / 1e9 for a, b, _ in intervals for t in (a, (a + b) / 2, b)])
        row = dict(name=name, requested_interval_ns=[original_left, original_right],
            observed_interval_ns=[left, right], excluded_suffix_ns=original_right - right,
            emitted_vx_integral_m=sum(c[0] * (b-a) / 1e9 for a, b, c in intervals),
            emitted_wz_integral_rad=sum(c[5] * (b-a) / 1e9 for a, b, c in intervals),
            nonzero_emission_duration_sim_s=sum((b-a) / 1e9 for a, b, c in intervals if any(c)),
            max_abs_vx=max(abs(c[0]) for _, _, c in intervals),
            max_abs_wz=max(abs(c[5]) for _, _, c in intervals))
        transform = data["region_evaluation"]["initial_fixed_SE3_evaluation_only"]
        for source, poses, xf, gap in (("SLAM", data["poses"], None, .2),
                ("GT_only", data["truth"], transform, .15)):
            p, rotation, bracket = motion.positions(poses, sample_times, xf, gap)
            delta = p[:, 2] - p[:, 0]
            body = np.einsum("nij,nj->ni", np.transpose(rotation[:, 1], (0, 2, 1)), delta)
            yaw = np.arctan2(rotation[:, :, 1, 0], rotation[:, :, 0, 0])
            dyaw = np.arctan2(np.sin(yaw[:, 2]-yaw[:, 0]), np.cos(yaw[:, 2]-yaw[:, 0]))
            row[source] = dict(body_forward_m=float(body[:, 0].sum()),
                body_lateral_m=float(body[:, 1].sum()), yaw_delta_rad=float(dyaw.sum()),
                delta_camera_init_m=delta.sum(axis=0).tolist(), max_bracket_s=bracket)
        metrics.append(row)
    stops = [e for e in guard["events"] if e["event"] == "exact_zero_published"]
    result = dict(scope=__doc__, run=str(run), original_passed=data["passed"],
        original_failure=data["failure"], origin_stamp_ns=data["origin_stamp_ns"],
        terminal_stamp_ns=data["terminal_stamp_ns"], active_max_raw_tilt=data["active_max_imu_tilt"],
        first_raw_crossings_and_native_boundary=event_evidence, guard_stop_events=stops,
        native_stop_return_edges_short=edges, short_window_native_motion=metrics,
        initial_single_SE3_GT_only=transform, goal8_entered=False,
        metadata_guard_dropped=guard["dropped"],
        limitations=["Native command log sim times are cached floats, not Headers; raw IMU/SLAM use original integer Headers.",
            "Phase provenance is sampled actual NAV status or exact guard callback; receiver timing uncertainty remains.",
            "No GT control, extrapolation, source backfill, applied torque, counterfactual motion or sole mechanical causal claim.",
            "Post-terminal 3.1395 tilt is excluded from the original active maximum."],
        input_sha256={n:motion.sha(run/n) for n in ("first_eight_result.json",
            "nav_drift_guard.json", "joint_stop_adapter.jsonl", "process_cleanup.json")})
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(dict(sha256=motion.sha(output), windows=metrics,
        events=[(r["name"], r["native_at_event"]["sim"], r["native_at_event"]["value"],
            None if r["native_next_zero"] is None else r["native_next_zero"]["sim"],
            r["phase_before"]["phase"], r["phase_after"]["phase"])
            for r in event_evidence]), indent=2))


if __name__ == "__main__":
    audit(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
