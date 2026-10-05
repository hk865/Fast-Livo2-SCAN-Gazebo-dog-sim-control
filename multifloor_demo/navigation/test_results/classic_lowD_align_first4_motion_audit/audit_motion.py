"""Owned-clean first4 profile: native emitted velocity/SLAM response and handoffs.

No ROS, CDR or control. The original single fixed SE3 is evaluation-only.
Any PASS/FAIL and original receipt windows are retained exactly.
"""
import bisect
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def at_ns(row):
    return round(row["sim"] * 1e9)


def positions(rows, times, transform=None, max_gap=.2):
    unique = {row["stamp_ns"]:row for row in rows}
    ordered = [unique[key] for key in sorted(unique)]
    t = np.array([row["stamp_ns"] / 1e9 for row in ordered])
    p = np.array([row["p"] for row in ordered])
    assert t[0] <= times.min() and times.max() <= t[-1]
    indices = np.clip(np.searchsorted(t, times, side="right"), 1, len(t) - 1)
    bracket_gap = float(np.max(t[indices] - t[indices - 1]))
    assert bracket_gap <= max_gap + 1e-9, (bracket_gap, max_gap)
    p = np.stack([np.interp(times, t, p[:, axis]) for axis in range(3)], axis=1)
    rot = Slerp(t, Rotation.from_quat([row["q"] for row in ordered]))(times).as_matrix()
    if transform is not None:
        r = np.array(transform["rotation_world_from_slam"])
        p = (p - np.array(transform["translation"])) @ r
        rot = np.einsum("ij,njk->nik", r.T, rot)
    return p.reshape(-1, 3, 3), rot.reshape(-1, 3, 3, 3), bracket_gap


def audit(run, output):
    cleanup = json.loads((run / "process_cleanup.json").read_text())
    assert cleanup["owned_group_clean"]
    assert not output.exists(), "Refuse to overwrite an existing audit"
    data = json.loads((run / "first_four_result.json").read_text())
    assert data["origin_stamp_ns"] is not None, "No actual motion/calibration: do not invent a motion result"
    assert data["terminal_stamp_ns"] is not None
    status = [row for row in data["statuses"] if row["status"].get("request_id") == data["request_id"]]
    terminal = next((row for row in reversed(status) if row["status"]["state"] in ["failed","complete"]),None)
    requested_start,requested_end=data["origin_stamp_ns"],data["terminal_stamp_ns"]
    # A diagnostic does not extrapolate a native pose to a later driver status.
    # Original receipts/results remain full and immutable; excluded intervals
    # are disclosed separately rather than fabricating terminal movement.
    start=max(requested_start,min(r["stamp_ns"] for r in data["poses"]),min(r["stamp_ns"] for r in data["truth"]))
    end=min(requested_end,max(r["stamp_ns"] for r in data["poses"]),max(r["stamp_ns"] for r in data["truth"]))
    assert start<end
    observation_boundary=dict(requested_ns=[requested_start,requested_end],observed_ns=[start,end],
        excluded_prefix_ns=start-requested_start,excluded_suffix_ns=requested_end-end,
        reason="Common native pose support; no extrapolation. Original terminal and original region windows unchanged.")
    definitions=data["goals_definitions"]
    assert len(definitions)==4, "This audit is explicitly first4, not a changed task"
    receipts=[row["receipt"] for row in data["region_evaluation"]["regions"] if row.get("receipt")]
    adapter = []
    relevant = {"actual_champ_command", "zero_edge", "native_zero_ack", "return_complete"}
    with (run / "joint_stop_adapter.jsonl").open() as stream:
        for seq, line in enumerate(stream):
            row = json.loads(line)
            if row["kind"] in relevant:
                row["native_file_line"] = seq + 1
                adapter.append(row)
    commands = [row for row in adapter if row["kind"] == "actual_champ_command"]
    assert all(a["sim"] <= b["sim"] for a, b in zip(commands, commands[1:]))
    intervals = []
    for a, b in zip(commands, commands[1:]):
        l, h = max(start / 1e9, a["sim"]), min(end / 1e9, b["sim"])
        if h <= l:
            continue
        c = a["value"]
        mode = "walk" if c[0] != 0 or c[1] != 0 else "turn" if c[5] != 0 else "zero"
        intervals.append((l, h, mode, c))
    assert intervals and intervals[0][0] == start / 1e9 and intervals[-1][1] == end / 1e9
    times = np.array([t for a, b, _, _ in intervals for t in (a, (a + b) / 2, b)])
    transform = data["region_evaluation"]["initial_fixed_SE3_evaluation_only"]
    motion = {}
    gaps = {}
    for name, rows, trans, gap in [("slam", data["poses"], None, .2),
                                 ("truth_evaluation_only", data["truth"], transform, .15)]:
        p, rot, actual_gap = positions(rows, times, trans, gap)
        dp = p[:, 2] - p[:, 0]
        body_dp = np.einsum("nij,nj->ni", np.transpose(rot[:, 1], (0, 2, 1)), dp)
        yaw = np.arctan2(rot[:, :, 1, 0], rot[:, :, 0, 0])
        dyaw = np.arctan2(np.sin(yaw[:, 2] - yaw[:, 0]), np.cos(yaw[:, 2] - yaw[:, 0]))
        motion[name] = (dp, body_dp, dyaw)
        gaps[name] = actual_gap

    def total(indices):
        out = dict(duration_sim_s=sum(intervals[j][1] - intervals[j][0] for j in indices),
            emitted_command_vx_integral_m=sum(intervals[j][3][0] * (intervals[j][1] - intervals[j][0]) for j in indices),
            emitted_command_wz_integral_rad=sum(intervals[j][3][5] * (intervals[j][1] - intervals[j][0]) for j in indices),
            emitted_vx_at_cap_duration_s=sum(intervals[j][1] - intervals[j][0] for j in indices if intervals[j][3][0] >= .119),
            emitted_yaw_at_walk_cap_duration_s=sum(intervals[j][1] - intervals[j][0] for j in indices
                if intervals[j][2] == "walk" and abs(intervals[j][3][5]) >= .079))
        positive_vx=[j for j in indices if intervals[j][2]=="walk" and intervals[j][3][0]>0]
        same_sign_walk_yaw={str(sign):[j for j in positive_vx if sign*intervals[j][3][5]>.005] for sign in [-1,1]}
        out["walk_yaw_sign_groups"]={}
        for sign,subset in same_sign_walk_yaw.items():
            out["walk_yaw_sign_groups"][sign]=dict(duration_sim_s=sum(intervals[j][1]-intervals[j][0] for j in subset),
                emitted_yaw_integral_rad=sum(intervals[j][3][5]*(intervals[j][1]-intervals[j][0]) for j in subset),
                slam_yaw_delta_rad=float(np.sum(motion["slam"][2][subset])),
                GT_evaluation_yaw_delta_rad=float(np.sum(motion["truth_evaluation_only"][2][subset])))
        for name, (dp, body, dyaw) in motion.items():
            out[name] = dict(delta_camera_init_m=np.sum(dp[indices], axis=0).tolist(),
                local_body_forward_m=float(np.sum(body[indices, 0])), yaw_delta_rad=float(np.sum(dyaw[indices])))
        return out

    mode_sums = {mode:total([j for j, row in enumerate(intervals) if row[2] == mode]) for mode in ("walk", "turn", "zero")}
    segments = []
    j = 0
    while j < len(intervals):
        k = j + 1
        while k < len(intervals) and intervals[k][2] == intervals[j][2]:
            k += 1
        segments.append(dict(mode=intervals[j][2], start=intervals[j][0], end=intervals[k - 1][1],
                             **total(list(range(j, k)))))
        j = k

    guard = json.loads((run / "nav_drift_guard.json").read_text())
    trajectory = [json.loads(line) for line in (run / "feedback_navigation_trajectories.jsonl").open()]
    events = guard["events"]
    stop_indices = [j for j, row in enumerate(events) if row["event"] == "exact_zero_published"]
    edges = [j for j, row in enumerate(adapter) if row["kind"] == "zero_edge"]
    poses = {row["stamp_ns"]: row for row in data["poses"]}
    handoffs = []
    for count, i in enumerate(stop_indices):
        stop = events[i]
        next_stop = stop_indices[count + 1] if count + 1 < len(stop_indices) else len(events)
        later = events[i + 1:next_stop]
        ref = next((row for row in later if row["event"] == "fresh_reference_requested"),None)
        accept = next((row for row in later if row["event"] == "fresh_checked_path_accepted"),None)
        if ref is None or accept is None:
            handoffs.append(dict(goal_index=stop["waypoint_index"],stop_ns=stop["callback_clock_ns"],
                completed_handoff=False,reason="No recorded fresh reference/checked path before terminal",
                checks=dict(complete_recorded_handoff=False)))
            continue
        trigger = stop["trigger"]
        edge_i = edges[stop["adapter_stop_count_before"]]
        edge_end = edges[stop["adapter_stop_count_before"] + 1] if stop["adapter_stop_count_before"] + 1 < len(edges) else len(adapter)
        section = adapter[edge_i:edge_end]
        preceding = next(row for row in reversed(adapter[:edge_i]) if row["kind"] == "actual_champ_command")
        ack = next(row for row in section if row["kind"] == "native_zero_ack")
        idle = next(row for row in section if row["kind"] == "return_complete")
        resume_i = next(j for j in range(edge_i, len(adapter)) if adapter[j]["kind"] == "actual_champ_command" and any(adapter[j]["value"]))
        held = [row for row in adapter[edge_i:resume_i] if row["kind"] == "actual_champ_command"]
        md = next(row["metadata"] for row in trajectory if row["kind"] == "scan_metadata"
            and row["metadata"]["reference_stamp"] == ref["reference_stamp"]
            and row["metadata"]["trajectory"]["traj_id"] == accept["identity"]["traj_id"])
        v = md["trajectory"]
        knots = np.array(v["knots"])
        order = v["order"]
        points = BSpline(knots, v["pos_pts"], order)(np.linspace(knots[order], knots[-order - 1],
            min(2000, max(2, int((knots[-order - 1] - knots[order]) / .08) + 1))))
        paths = [row for row in trajectory if row["kind"] == "accepted_path" and row["points"]
            and abs(round(row["receipt_sim_time"] * 1e9) - accept["callback_clock_ns"]) <= 30_000_000
            and np.shape(row["points"]) == points.shape]
        error = min(float(np.max(np.abs(np.array(row["points"]) - points))) for row in paths)
        checks = dict(actual_raw_trigger=trigger["stamp_ns"] in poses and np.array_equal(
            trigger["raw_position"], poses[trigger["stamp_ns"]]["p"]),
            native_zero_first=not any(preceding["value"]),
            all_native_emits_until_resume_zero=bool(held) and all(not any(row["value"]) for row in held),
            native_stop_ack_and_return=ack["kind"] == "native_zero_ack" and idle["kind"] == "return_complete",
            actual_idle_before_zero_window=at_ns(idle) <= ref["handoff"]["actual_zero_since_ns"],
            fresh_full_metadata_same_goal=md["reference_stamp"] == accept["identity"]["reference_stamp"]
                and np.allclose(md["body_goal"], ref["handoff"]["body_goal"], rtol=0., atol=1e-8),
            original_published_path_reconstructed=error <= 1e-12,
            native_motion_after_checked_path_and_one_second_pre=at_ns(adapter[resume_i]) >= accept["callback_clock_ns"] + 1_000_000_000)
        handoffs.append(dict(completed_handoff=True,goal_index=stop["waypoint_index"], trigger_ns=trigger["stamp_ns"],
            stop_ns=stop["callback_clock_ns"], actual_zero_native=preceding,
            native_zero_ack=ack, native_idle=idle, reference_ns=ref["callback_clock_ns"],
            checked_accept_ns=accept["callback_clock_ns"], first_native_motion=adapter[resume_i],
            native_zero_emissions_before_resume=len(held), accepted_path_max_error=error,
            checks={key:bool(value) for key, value in checks.items()}))

    goal_motion=[]
    bounds=[start]+[r["stamp_ns"] for r in receipts]
    if bounds[-1]<end:bounds.append(end)
    for goal_index,(l,h) in enumerate(zip(bounds,bounds[1:])):
        selected=[j for j,(a,b,_,_) in enumerate(intervals) if a>=l/1e9 and b<=h/1e9]
        # Global native intervals can straddle receipt boundaries. These groups
        # are diagnostic only; exclude the crossing interval and disclose it.
        excluded=sum(max(0.,min(b,h/1e9)-max(a,l/1e9)) for j,(a,b,_,_) in enumerate(intervals) if j not in selected)
        goal_motion.append(dict(goal_index=goal_index if goal_index<len(definitions) else None,
            phase="goal" if goal_index<len(definitions) else "post_final_receipt_tail",interval_ns=[l,h],
            bound_source="origin/prior original receipt -> next original receipt or actual terminal",
            excluded_crossing_command_interval_s=excluded,
            sums_by_mode={mode:total([j for j in selected if intervals[j][2]==mode]) for mode in ["walk","turn","zero"]}))
    names = ["first_four_result.json", "joint_stop_adapter.jsonl", "nav_drift_guard.json",
             "feedback_navigation_trajectories.jsonl", "process_cleanup.json", "control_parameter_readback.json",
             "staging/nav_align_controller.py", "staging/turn_drift.py", "staging/align_translation.py",
             "nav_align_configuration_readback.json"]
    result = dict(scope=__doc__, run=str(run), original_passed=data["passed"], original_failure=data.get("failure"),
        original_native_region_evaluation=data["region_evaluation"],goal_motion=goal_motion,
        checks=dict(original_result_retained=True, owned_clean=cleanup["owned_group_clean"],
            all_native_handoffs=all(all(row["checks"].values()) for row in handoffs)
                and len(handoffs) == guard["interruptions"],
            complete_observed_command_interval=abs(sum(row[1] - row[0] for row in intervals) - (end - start) / 1e9) < 1e-8),
        terminal_goal_index=None if terminal is None else terminal["status"].get("waypoint_index"),
        original_goal_definitions=definitions,
        mode_interval_ns=[start, end],observation_boundary=observation_boundary,
        mode_interval_source="original active interval intersected with actual common SLAM/GT support; transitions included",
        single_original_fixed_SE3_evaluation_only=transform, observed_max_interpolation_gap_s=gaps,
        mode_sums=mode_sums, mode_segments=segments, handoffs=handoffs,
        actual_terminal_status={key:None if terminal is None else terminal["status"].get(key) for key in ["message", "pose", "command", "steering",
            "alignment_phase", "locked_heading", "accepted_trajectory_id", "turn_drift_guard"]},
        input_sha256={name:sha(run / name) for name in names},
        limitations=["No GT input to interruption or control; GT only evaluates motion with the single original initial SE3.",
            "Native Adapter velocity emission is publisher evidence, not actual CHAMP receive, applied torque or no-slip motion.",
            "Command sim times are cached float-clock log fields, not message Headers. Bracketed SLAM/GT boundaries use Slerp and position interpolation without extrapolation.",
            "This validates association/full published path reconstruction; collision optimization is not independently rerun from occupancy.",
            "No JTC effort, ResourceManager clipping or joint/contact mechanism is inferred here. Original component results remain unchanged.",
            "A same-sign walk-yaw aggregate is a descriptive grouped integral, not a constant-command plant identification or proof of one causal sign reversal.",
            "Goal-wise sums disclose omitted intervals straddling original receipt edges; all-observed totals cover the disclosed native pose-supported interval."])
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(dict(run=run.name,checks=result["checks"],original_passed=data["passed"],modes=mode_sums,
        handoffs=len(handoffs),report_sha256=sha(output)),indent=2))


if __name__ == "__main__":
    run=Path(sys.argv[1]).resolve()
    output=Path(sys.argv[2]).resolve() if len(sys.argv)>2 else HERE/(run.name+"_motion.json")
    audit(run,output)
