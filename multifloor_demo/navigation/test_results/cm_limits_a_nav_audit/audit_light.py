"""Bounded NAV-only audit while the following physical run is active.

Reads the first 1 MB of the result, last 300 kB of event statuses and the two
small guard/trajectory records. Does not decode CDR or scan the native adapter,
sensor, truth or feedback files. A component failure remains a failure.
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "navigation"))
from goal_regions import contains, contains_control, definitions_sha256, parse_goal


def sha_bytes(data):
    return hashlib.sha256(data).hexdigest()


def prefix_fields(path):
    with path.open("rb") as stream:
        data = stream.read(1_000_000)
    text = data.decode()
    decoder = json.JSONDecoder()
    index = 1
    fields = {}
    while index < len(text):
        while index < len(text) and text[index] in " ,\r\n\t":
            index += 1
        if text[index:index + 1] == "}":
            break
        key, index = decoder.raw_decode(text, index)
        index += 1
        try:
            value, end = decoder.raw_decode(text, index)
        except json.JSONDecodeError:
            return fields, data, key
        fields[key] = value
        index = end
    return fields, data, None


def main():
    run = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "simulation/test_results/20261002_cm_limits_first8_a_false"
    original, prefix, excluded_next = prefix_fields(run / "first_eight_result.json")
    cleanup_bytes = (run / "process_cleanup.json").read_bytes()
    cleanup = json.loads(cleanup_bytes)
    assert cleanup["owned_group_clean"] is True
    guard_bytes = (run / "nav_drift_guard.json").read_bytes()
    guard = json.loads(guard_bytes)
    trajectory_bytes = (run / "feedback_navigation_trajectories.jsonl").read_bytes()
    trajectories = [json.loads(line) for line in trajectory_bytes.splitlines()]
    metadata = [row for row in trajectories if row["kind"] == "scan_metadata"]
    event_path = run / "first_eight_events.jsonl"
    with event_path.open("rb") as stream:
        offset = max(0, event_path.stat().st_size - 300_000)
        stream.seek(offset)
        tail = stream.read()
    tail_rows = [json.loads(line) for line in tail.splitlines()[1:]]
    statuses = [row for row in tail_rows if row["kind"] == "actual_navigation_status"]
    failed = [row for row in statuses if row["data"]["state"] == "failed"]
    goals = [parse_goal(definition) for definition in original["goals_definitions"]]
    poses = {row["stamp_ns"]: row for row in original["poses"]}
    regions = []
    prior = original["origin_stamp_ns"]
    for index, report in enumerate(original["region_evaluation"]["regions"]):
        receipt = report.get("receipt")
        if not receipt:
            regions.append(dict(index=index, goal_id=goals[index].goal_id,
                                original_receipt_present=False, original_passed=False))
            continue
        observations = report["observations"]
        stamps = [row["stamp_ns"] for row in observations]
        checks = dict(
            original_receipt_present=True,
            original_receipt_identity=receipt["request_id"] == original["request_id"]
                and receipt["goal_id"] == goals[index].goal_id
                and receipt["waypoint_index"] == index,
            complete_definitions_hash=receipt["goals_definition_sha256"]
                == original["goals_definition_sha256"],
            ordered_fresh_window=receipt["start_stamp_ns"] > prior,
            actual_ns_window=stamps[0] == receipt["start_stamp_ns"]
                and stamps[-1] == receipt["stamp_ns"],
            original_dwell=receipt["dwell_ns"] >= 400_000_000,
            original_max_gap=bool(max(np.diff(stamps), default=0) <= 200_000_000),
            unprotected=receipt["protected"] is False,
            receipt_raw_matches_recorded_odom=all(
                row["stamp_ns"] in poses and np.array_equal(
                    row["raw_slam"], poses[row["stamp_ns"]]["p"])
                for row in observations),
            raw_entire_window_inner=all(contains_control(goals[index], row["raw_slam"])
                for row in observations),
            reported_fixed_SE3_truth_entire_window_outer=all(
                contains(goals[index], row["truth_in_slam"]) for row in observations),
            receipt_before_original_90_deadline=(receipt["stamp_ns"] - prior) <= 90_000_000_000,
        )
        regions.append(dict(index=index, goal_id=goals[index].goal_id,
            original_receipt_present=True, original_passed=report["passed"],
            window_ns=[receipt["start_stamp_ns"], receipt["stamp_ns"]],
            elapsed_from_prior_receipt_s=(receipt["stamp_ns"] - prior) / 1e9,
            checks=checks))
        prior = receipt["stamp_ns"]

    def records(name):
        return [row for row in guard["events"] if row["event"] == name]

    handoffs = []
    references = records("fresh_reference_requested")
    accepted = records("fresh_checked_path_accepted")
    for stop in records("exact_zero_published"):
        trigger = stop["trigger"]
        ref = next(row for row in references if row["callback_clock_ns"] > stop["callback_clock_ns"])
        accept = next(row for row in accepted if row["callback_clock_ns"] > ref["callback_clock_ns"])
        identity = accept["identity"]
        matching = [row for row in metadata if
            row["metadata"]["reference_stamp"] == ref["reference_stamp"]
            and row["metadata"]["trajectory"]["traj_id"] == identity["traj_id"]]
        ack = ref["actual_adapter_ack"]
        checks = dict(
            raw_trigger_matches_recorded_odom=trigger["stamp_ns"] in poses
                and np.array_equal(trigger["raw_position"], poses[trigger["stamp_ns"]]["p"]),
            original_displacement_and_dwell=trigger["raw_planar_drift_m"] >= .15
                and trigger["persistence_ns"] >= 200_000_000
                and trigger["raw_observations"] >= 3,
            zero_requested_before_new_reference=stop["command"] == [0., 0., 0.],
            same_actual_goal_index=stop["waypoint_index"] == ref["waypoint_index"]
                == accept["waypoint_index"],
            fresh_idle_ack=ack["state"] == "idle" and not ack["failed"]
                and ack["nominal_calibrated"]
                and ack["counters"]["stops"] > stop["adapter_stop_count_before"]
                and round(ack["sim"] * 1e9) > stop["callback_clock_ns"],
            original_one_second_idle_before_new_reference=ref["callback_clock_ns"]
                - ref["handoff"]["actual_zero_since_ns"] >= 1_000_000_000,
            upstream_requested_safe_zero=ref["bridge_safe"] == [0., 0., 0.]
                and ref["bridge_requested"] == [0., 0., 0.],
            full_metadata_identity=bool(matching)
                and identity["reference_stamp"] == ref["reference_stamp"]
                and np.allclose(identity["body_goal"], goals[stop["waypoint_index"]].center,
                                rtol=0., atol=1e-12)
                and all(np.allclose(row["metadata"]["body_goal"], identity["body_goal"],
                                    rtol=0., atol=1e-12)
                        for row in matching),
        )
        handoffs.append(dict(goal_index=stop["waypoint_index"],
            trigger_ns=trigger["stamp_ns"], zero_callback_ns=stop["callback_clock_ns"],
            reference_ns=ref["callback_clock_ns"], accept_ns=accept["callback_clock_ns"],
            checks=checks))

    active = [row for row in original["poses"] if prior <= row["stamp_ns"] <= original["terminal_stamp_ns"]]
    current_index = statuses[-1]["data"]["waypoint_index"]
    current = goals[current_index]
    distances = [float(np.linalg.norm(np.array(row["p"]) - current.center)) for row in active]
    last_status = statuses[-1]["data"]
    terminal = dict(
        original_state=original["passed"], original_failure=original["failure"],
        terminal_ns=original["terminal_stamp_ns"], failed_status=[dict(
            receive_ns=row["received_stamp_ns"], state=row["data"]["state"],
            index=row["data"]["waypoint_index"], message=row["data"]["message"])
            for row in failed],
        prior_receipt_ns=prior, elapsed_from_prior_receipt_s=(original["terminal_stamp_ns"] - prior) / 1e9,
        active_goal_index=current_index, active_goal_id=current.goal_id,
        eighth_goal_never_entered=original["eighth_segment_entry_stamp_ns"] is None,
        closest_raw_center_m=min(distances),
        raw_inner_samples=sum(contains_control(current, row["p"]) for row in active),
        last_raw_pose=active[-1], last_raw_center_m=distances[-1],
        last_navigation_counters={key:last_status[key] for key in [
            "replans", "degenerate_splines", "trajectory_association_rejected",
            "obstacle_stops", "tilt_stops", "accepted_trajectory_id", "alignment_phase"]},
        active_max_raw_tilt=original["active_max_imu_tilt"],
        active_bridge_holds=original["active_bridge_holds"],
        first_tilt_hold_context="not applicable: no active tilt hold",
    )
    result = dict(
        scope="Bounded NAV audit; not a component PASS or a native actuator/JTC response audit",
        run=str(run), original_failure_preserved=original["passed"] is False,
        checks=dict(owned_clean=cleanup["owned_group_clean"],
            canonical_definitions_hash=definitions_sha256(goals) == original["goals_definition_sha256"],
            actual_completed_raw_region_receipts_valid=all(all(row["checks"].values())
                for row in regions if row["original_receipt_present"]),
            guard_record_has_no_drops=guard["dropped"] == 0,
            all_logged_handoff_contracts_valid=len(handoffs) == guard["interruptions"]
                and all(all(row["checks"].values()) for row in handoffs)),
        regions=regions, handoffs=handoffs, terminal=terminal,
        provenance=dict(result_read_prefix_bytes=len(prefix), result_prefix_sha256=sha_bytes(prefix),
            next_unread_result_field=excluded_next,
            guard_full_sha256=sha_bytes(guard_bytes), trajectories_full_sha256=sha_bytes(trajectory_bytes),
            cleanup_full_sha256=sha_bytes(cleanup_bytes), status_suffix_bytes=len(tail),
            status_suffix_sha256=sha_bytes(tail)),
        limitations=[
            "The GT coordinates in six original receipt reports were rechecked geometrically; original truth stream re-pairing is deferred to the independent reviewer.",
            "Bridge.safe is upstream safe command, not applied effort or actual robot velocity.",
            "Native adapter emission/return trajectories and full accepted-path reconstruction are deferred until B is owned clean.",
            "The elapsed prior receipt includes goal-transition scheduling; exact segment_started_ros is not present in this bounded record.",
            "Metadata body-goal versus declared center allows 1e-12 m for the body-height floating-point transform; reference stamp and trajectory id remain exact.",
            "No CDR/JTC/q/sensor/command response decoding and no new ROS or physics.",
        ],
    )
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).with_name("light_audit.json")
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(dict(checks=result["checks"], receipt_windows=[row.get("window_ns")
        for row in regions], terminal=terminal), ensure_ascii=False))


if __name__ == "__main__":
    main()
