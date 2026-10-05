"""Actual new gain/BOOL/cadence/finite effort contract; no plant fit or adoption."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

attempt = Path((HERE / "latest_attempt.txt").read_text().strip())
execution = json.loads((attempt / "execution_receipt.json").read_text())
rows = [json.loads(line) for line in (attempt / "true_batch1.jsonl").open()]
config = next(row for row in rows if row["kind"] == "configuration")
service = next(row for row in rows if row["kind"] == "parameter_service")
gain_service = next(row for row in rows if row["kind"] == "all_joint_gain_service")
pid_gains = next(row for row in rows if row["kind"] == "actual_pid_gains")
preset = next(row for row in rows if row["kind"] == "prescribed_conditions")
initial = next(row for row in rows if row["kind"] == "sample" and row["stage"] == "activation")
samples = [row for row in rows if row["kind"] == "sample" and row["stage"] != "activation"]
inputs = [row for row in rows if row["kind"] == "actual_input"]
names = [row["joint"] for row in gain_service["joint_values"]]
gain_expected = dict(p=220.982919, i=.2, d=1.0, i_clamp=2.5, ff_velocity_scale=0.)
residuals = []
stage_stats = {}
for stage in dict.fromkeys(row["stage"] for row in samples):
    selected = [row for row in samples if row["stage"] == stage]
    values = np.array([row["output_effort"] for row in selected])
    stage_stats[stage] = dict(updates=len(selected), first_time_ns=selected[0]["time_ns"],
        last_time_ns=selected[-1]["time_ns"], output_abs_peak=float(np.max(np.abs(values))),
        actual_next_effort_abs_peak=float(np.max(np.abs([row["next_effort"] for row in selected]))))
for row in samples:
    p = np.array(row["actual_pid_p_error"])
    d = np.array(row["actual_pid_d_error"])
    i = np.array(row["actual_pid_i_weighted"])
    pid = np.array(row["actual_pid_command"])
    ff = np.array(row["next_effort"])
    output = np.array(row["output_effort"])
    residuals.append(float(max(np.max(np.abs(pid - (gain_expected["p"] * p + gain_expected["d"] * d + i))),
                               np.max(np.abs(output - (ff + pid))))))

base = initial["time_ns"]
nominal = np.array(preset["nominal"])
changed = nominal + np.array([(-1. if j % 2 == 0 else 1.) * .01 for j in range(12)])
restarted = nominal + np.array([(-1. if j % 2 == 0 else 1.) * .005 for j in range(12)])
input_errors = []
for row in inputs:
    t = row["time_ns"] - base
    expected = nominal
    if 1_200_000_000 < t <= 1_500_000_000:
        expected = changed
    elif 1_500_000_000 < t <= 1_875_000_000:
        s = (t - 1_500_000_000) / 375_000_000
        alpha = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        expected = changed + alpha * (nominal - changed)
    elif t > 2_000_000_000:
        expected = restarted
    input_errors.append(float(np.max(np.abs(row["target"] - expected))))

maps = {}
for line in (attempt / "true_batch1.jsonl.maps").read_text().splitlines():
    path = line.split()[-1]
    if path.startswith("/") and any(key in path for key in ["libjoint_trajectory_controller.so", "libcontroller_interface.so",
                                                           "libcontrol_toolbox.so", "libhardware_interface.so"]):
        maps[path] = sha(Path(path))
numeric_arrays = ["hardware_position", "hardware_velocity", "output_effort", "actual_position", "actual_velocity",
    "reference_position", "reference_velocity", "next_position", "next_velocity", "next_effort", "actual_pid_p_error",
    "actual_pid_i_weighted", "actual_pid_d_error", "actual_pid_command"]
checks = dict(
    actual_flag_BOOL_true_and_closed_loop=config["actual_parameter"] is True and service["flag_type"] == 1
        and service["flag_bool"] is True and config["closed_loop_effort_pid"] is True
        and config["open_loop_control"] is False and service["open_loop_bool"] is False,
    actual_sync_JTC=config["is_async"] is False,
    actual_all12_gain_RPC=len(set(names)) == 12 and all(all(row[key] == value for key, value in gain_expected.items())
        for row in gain_service["joint_values"]),
    actual_all12_PID_gain_getters=all(len(pid_gains[key]) == 12 and all(value == expected for value in pid_gains[key])
        for key, expected in dict(p=220.982919, i=.2, d=1.0, i_clamp_max=2.5, i_clamp_min=-2.5).items()),
    actual_ff_velocity_scales_zero=preset["actual_ff_velocity_scales"] == [0.] * 12,
    activation_real_command_zero=initial["output_effort"] == [0.] * 12,
    separate_integer200Hz_producer_250Hz_update=len(inputs) == 441 and len(samples) == 550
        and all(b["time_ns"] - a["time_ns"] == 5_000_000 for a, b in zip(inputs, inputs[1:]))
        and all(b["time_ns"] - a["time_ns"] == 4_000_000 for a, b in zip(samples, samples[1:])),
    all_actual_new_callback_confirmed_header0_position_only=all(row["new_callback_observed"]
        and row["header_ns"] == 0 and row["horizon_ns"] == 16_666_666 for row in inputs),
    actual_returned_sync_periods_4ms=all(row["actual_returned_period_ns"] == 4_000_000 and row["actual_returned_success"] for row in samples),
    same_prescribed_actual_measured_states=all(row["hardware_position"] == initial["hardware_position"]
        and row["hardware_velocity"] == initial["hardware_velocity"]
        and row["actual_position"] == row["hardware_position"] and row["actual_velocity"] == row["hardware_velocity"] for row in samples),
    actual_PID_plus_actual_reference_FF_reconstructed=max(residuals) < 1e-11,
    actual_reference_FF_always_zero=all(value == 0. for row in [initial, *samples] for value in row["next_effort"]),
    all_observed_vectors_finite=all(np.isfinite(row[key]).all() for row in [initial, *samples] for key in numeric_arrays),
    producer_anchored_changed_return_restart_same_input_formula=max(input_errors) < 1e-12,
    all5_stages_exercised=set(stage_stats) == {"quasistatic", "changed_target", "producer_stop_return", "nominal_idle", "rapid_restart"},
    actual12_weighted_I_bounded=all(abs(value) <= 2.5 for row in samples for value in row["actual_pid_i_weighted"]),
    current_installed_JTC_interface_toolbox_mapped=len(maps) == 3
        and all(any(name in path for path in maps) for name in ["libjoint_trajectory_controller.so",
            "libcontroller_interface.so", "libcontrol_toolbox.so"]),
    owned77_clean=execution["execution"]["exit_code"] == 0 and execution["execution"]["owned_group_clean"] and not execution["execution"]["timeout"],
    production269_unchanged=execution["production_source_count"] == 269
        and execution["production_fingerprint_match_before"] and execution["production_fingerprint_match_after"]
        and not execution["production_changes"],
)
checks = {key:bool(value) for key, value in checks.items()}
hardware_headers = [Path("/opt/ros/jazzy/include/hardware_interface/hardware_interface/loaned_command_interface.hpp"),
                    Path("/opt/ros/jazzy/include/hardware_interface/hardware_interface/loaned_state_interface.hpp")]
result = dict(scope=__doc__, passed=all(checks.values()), checks=checks, named_checks=len(checks),
    selected_gain_profile=gain_expected, actual_flag=config, actual_all_joint_RPC=gain_service,
    actual_PID_gain_getters=pid_gains, actual_loaded_libraries=maps, stages=stage_stats,
    actual_PID_plus_FF_max_residual=max(residuals), actual_producer_return_formula_max_error=max(input_errors),
    controlled_cadence=dict(producer_period_ns=5_000_000, update_period_ns=4_000_000,
        tie_order="confirmed real DDS trajectory callback before update", wall_cadence_not_asserted=True,
        actual_inputs=len(inputs), actual_updates=len(samples), prescribed_states_not_physical_trajectory=True),
    execution=execution, source_sha256=execution["source_sha256"],
    actual_analysis_sha256=sha(Path(__file__)),
    memory_interface_installed_header_sha256={str(path):sha(path) for path in hardware_headers},
    input_sha256={path.name:sha(path) for path in attempt.iterdir() if path.is_file()
        and path.name not in ["result.json", "analysis_used.py"]},
    limitations=["This is a bare actual installed JTC with controlled memory interfaces, not Gazebo or a stable Go2 plant test.",
        "Output effort is a command-interface value, not measured applied torque. This fixture does not instantiate CM/ResourceManager or URDF final command limits.",
        "CM enforce=true total-limit mechanism is covered by the separately reviewed native RM contract; no CM Boolean service is claimed in this process.",
        "Prescribed constant q with independently preset nonzero v is an interface snapshot, not a physically consistent trajectory.",
        "FF zero is observed for zero initial command and this position-only activation/changed/return/restart stream; no arbitrary inherited live hardware state is claimed.",
        "No refitted gains, adoption, new physics or production edits. Both historical CM A/B FAIL results remain unchanged."])
(attempt / "analysis_used.py").write_bytes(Path(__file__).read_bytes())
(attempt / "result.json").write_text(json.dumps(result, indent=2) + "\n")
(HERE / "final_receipt.json").write_text(json.dumps(dict(passed=result["passed"], named_checks=len(checks),
    result=str(attempt / "result.json"), result_sha256=sha(attempt / "result.json"),
    source_sha256=execution["source_sha256"], actual_loaded_libraries=maps), indent=2) + "\n")
print(json.dumps(dict(passed=result["passed"], named_checks=len(checks), checks=checks, stages=stage_stats,
    actual_PID_plus_FF_max_residual=max(residuals), result_sha256=sha(attempt / "result.json")), indent=2))
raise SystemExit(not result["passed"])
