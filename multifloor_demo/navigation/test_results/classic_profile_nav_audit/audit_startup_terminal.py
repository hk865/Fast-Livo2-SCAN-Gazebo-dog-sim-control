"""Owned-clean classic profile failed before calibration/request: bounded audit."""
import hashlib
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
RUN = ROOT / "simulation/test_results/20261002_classic_pd_first4_candidate"
reader_path = HERE.parent / "cm_limits_a_nav_audit/audit_light.py"
spec = importlib.util.spec_from_file_location("bounded_NAV_prefix_reader", reader_path)
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)

def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for data in iter(lambda:stream.read(1024 * 1024), b""):
            h.update(data)
    return h.hexdigest()

cleanup = json.loads((RUN / "process_cleanup.json").read_text())
assert cleanup["owned_group_clean"]
gate = json.loads((RUN / "startup_gate.json").read_text())
result, prefix, unread = reader.prefix_fields(RUN / "first_four_result.json")
checks = dict(
    original_FAIL_preserved=result["passed"] is False,
    actual_startup_gate_FAILED=gate["passed"] is False
        and gate["reason"] == "timeout_before_static_initialization: angular_motion",
    no_actual_origin_or_calibration=result["origin_stamp_ns"] is None and result["heading_alignment"] is None,
    no_actual_goals_or_arrival_windows=not result["goals_definitions"] and not result["region_evaluation"]["regions"],
    no_actual_SLAM_body_pose=result["poses"] == [],
    fourth_segment_never_entered=result["fourth_segment_entry_stamp_ns"] is None,
    no_parameter_gate_receipt=not (RUN / "control_parameter_readback.json").exists(),
    owned_clean=cleanup["owned_group_clean"] and cleanup["return_code"] == 0,
    original_sources_runtime_and_assets_unchanged=all(cleanup[key] for key in [
        "source_unchanged", "runtime_unchanged", "staging_unchanged", "control_profile_assets_unchanged", "sensor_assets_unchanged"]),
)
report = dict(scope=__doc__, audit_checks_passed=all(checks.values()), checks=checks,
    original_component_passed=False, original_failure=result["failure"],
    actual_startup_failure=gate, original_terminal_stamp_ns=result["terminal_stamp_ns"],
    actual_completed_region_windows=0, completed_region_acceptance="not available: request/calibration never occurred",
    actual_motion_audit="not available: no active SLAM frame or navigation goal",
    actual_loaded_CM_verification=cleanup["actual_selected_CM_verified"],
    actual_physical_control_profile_verification=cleanup["actual_control_profile_verified"],
    input_sha256={name:(sha(RUN / name) if (RUN / name).exists() else None) for name in ["first_four_result.json", "process_cleanup.json",
        "startup_gate.json", "preflight.json", "driver_exit.json", "source_manifest.json", "runtime_manifest.json", "cm_manifest.json"]},
    result_prefix_bytes=len(prefix), result_prefix_sha256=hashlib.sha256(prefix).hexdigest(), unread_next_field=unread,
    limitations=[
        "No native region window, initial fixed SE3 or NAV movement exists; absent values are not a passed check of physical navigation.",
        "The cleanup process return code 0 and unchanged source/assets do not erase startup failure or prove actual profile parameter readback.",
        "No CDR/JTC/contact analysis, no ROS node, no controller run, no counterfactual trajectory or production mutation.",
        "The independent bare-JTC native 19-check profile contract remains a separate interface test; it is not physical startup or stability evidence.",
    ])
(HERE / "startup_terminal_audit.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(dict(audit_checks_passed=report["audit_checks_passed"], checks=checks,
    reason=gate["reason"], original_component_passed=False,
    report_sha256=sha(HERE / "startup_terminal_audit.json")), indent=2))
