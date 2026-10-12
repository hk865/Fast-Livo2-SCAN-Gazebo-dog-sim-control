#!/usr/bin/env python3
"""Actual ROS IMU/joint histories for a separately controlled Teacher A/B.

Only subscribes to /livox/imu and /demo/control/measured_joint_states. It does
not load an actor, read truth telemetry, choose policy samples, or publish any
actuation/navigation command. The worker owns causal selection and fail-safe.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
TOPICS = {"imu": "/livox/imu", "joints": "/demo/control/measured_joint_states"}
HISTORY_SIZE = 80
EXPORTED_HISTORY_SIZE = 16
WRITE_PERIOD_S = .005
WALL_MAX_AGE_S = .3


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return data


def rotation(q):
    """Reference-from-local rotation, ROS quaternion xyzw, no guessed default."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .98 <= float(q @ q) <= 1.02:
        raise ValueError("invalid_orientation_quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def rpy_rotation(rpy):
    rpy = np.asarray(rpy, dtype=float)
    if rpy.shape != (3,) or not np.isfinite(rpy).all():
        raise ValueError("invalid_contract_rpy")
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.asarray([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]])


def stamp_ns(message):
    sec, ns = message.header.stamp.sec, message.header.stamp.nanosec
    if not isinstance(sec, int) or not isinstance(ns, int) or sec < 0 or not 0 <= ns < 1_000_000_000:
        raise ValueError("invalid_simulation_header_stamp")
    return sec*1_000_000_000 + ns


class FeedbackContract:
    def __init__(self, run):
        self.run = Path(run).resolve(strict=True)
        self.paths = {
            "sensor_contract": self.run / "sensor_contract.json",
            "asset_manifest": self.run / "asset_manifest.json",
            "policy_contract": self.run / "sources/policy/contract.json",
        }
        data = {key: read(path) for key, path in self.paths.items()}
        sensor, asset, policy = data["sensor_contract"], data["asset_manifest"], data["policy_contract"]
        self.hashes = {key: digest(path) for key, path in self.paths.items()}
        if sensor["imu"]["ros_topic"] != TOPICS["imu"] or sensor["joint_states"]["ros_topic"] != TOPICS["joints"]:
            raise ValueError("run_sensor_topics_do_not_match_fixed_actual_topics")
        self.imu_frame = sensor["imu"]["frame"]
        if not isinstance(self.imu_frame, str) or not self.imu_frame:
            raise ValueError("run_imu_frame_missing")
        reference = sensor["imu"]["orientation_reference"]
        if reference["localization"] != "CUSTOM" or reference["custom_parent_frame"] != "world":
            raise ValueError("unsupported_or_unestablished_imu_orientation_reference")
        self.body_imu = rotation(reference["body_imu_quaternion"])
        self.world_reference = rotation(reference["world_quaternion"])
        if not np.allclose(self.body_imu, rpy_rotation(sensor["imu"]["body_rpy"]), atol=1e-10):
            raise ValueError("run_imu_extrinsic_quaternion_rpy_conflict")
        if not np.allclose(self.world_reference, rpy_rotation(reference["world_rpy"]), atol=1e-10):
            raise ValueError("run_imu_reference_quaternion_rpy_conflict")
        self.joint_names = tuple(policy["gazebo_joint_names"])
        if len(self.joint_names) != 12 or len(set(self.joint_names)) != 12 or list(self.joint_names) != asset["joint_names"]:
            raise ValueError("run_joint_mapping_not_explicit_and_consistent")
        self.default_q = np.asarray(policy["default_joint_positions"], dtype=float)
        if self.default_q.shape != (12,) or not np.isfinite(self.default_q).all() or not np.array_equal(self.default_q, np.asarray(asset["default_q"])):
            raise ValueError("run_default_joint_positions_conflict")
        base_pose = np.fromstring(asset["base_inertial_pose"], sep=" ")
        if base_pose.shape != (6,) or not np.isfinite(base_pose).all():
            raise ValueError("run_base_inertial_pose_missing")
        self.description = {
            "run": str(self.run), "source_contract_paths": {key: str(path) for key, path in self.paths.items()},
            "source_contract_sha256": self.hashes, "imu_frame": self.imu_frame,
            "imu_orientation_reference": reference,
            "body_from_imu_rotation": self.body_imu.tolist(),
            "joint_order": list(self.joint_names), "default_q": self.default_q.tolist(),
            "actual_base_com_offset_body_m": base_pose[:3].tolist(),
            "base_com_scope": "Recorded provenance only; no linear velocity or COM estimate is constructed by this broker",
        }

    def assert_unchanged(self):
        if any(digest(path) != self.hashes[key] for key, path in self.paths.items()):
            raise ValueError("run_sensor_or_asset_contract_changed")

    def imu_values(self, message):
        if message.header.frame_id != self.imu_frame:
            raise ValueError("imu_frame_does_not_match_actual_run_contract")
        gyro = np.asarray([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
        cov_gyro = np.asarray(message.angular_velocity_covariance)
        cov_orientation = np.asarray(message.orientation_covariance)
        if not np.isfinite(gyro).all() or cov_gyro.shape != (9,) or not np.isfinite(cov_gyro).all() or cov_gyro[0] == -1:
            raise ValueError("imu_gyro_missing_invalid_or_unknown")
        if cov_orientation.shape != (9,) or not np.isfinite(cov_orientation).all() or cov_orientation[0] == -1:
            raise ValueError("imu_orientation_missing_invalid_or_unknown")
        q = message.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        world_body = self.world_reference @ rotation(quaternion) @ self.body_imu.T
        gravity = world_body.T @ np.asarray([0., 0., -1.])
        return {
            "gyro_body": (self.body_imu @ gyro).tolist(), "gravity_body": gravity.tolist(),
            "gyro_imu": gyro.tolist(), "orientation_xyzw": quaternion,
            "angular_velocity_covariance": cov_gyro.tolist(), "orientation_covariance": cov_orientation.tolist(),
            "orientation_source": "actual received Gazebo IMU CUSTOM/world orientation; no physical attitude-estimator validation",
        }

    def joint_values(self, message):
        names = list(message.name)
        if len(set(names)) != len(names):
            raise ValueError("joint_message_has_duplicate_names")
        indices = {name: i for i, name in enumerate(names)}
        if any(name not in indices for name in self.joint_names):
            raise ValueError("joint_message_missing_required_named_joint")
        order = [indices[name] for name in self.joint_names]
        try:
            q = np.asarray([message.position[i] for i in order])
            qd = np.asarray([message.velocity[i] for i in order])
        except IndexError as error:
            raise ValueError("joint_position_or_velocity_missing") from error
        if not np.isfinite(q).all() or not np.isfinite(qd).all():
            raise ValueError("joint_position_or_velocity_nonfinite")
        return {"q": q.tolist(), "qd": qd.tolist(), "ordered_names": list(self.joint_names),
                "message_names": names, "message_position": list(message.position), "message_velocity": list(message.velocity),
                "effort_scope": "Effort is not read or used; policy torque remains sole actuator last clipped command"}


def atomic_json(path, payload):
    temp = path.with_name(path.name + f".tmp.{os.getpid()}")
    try:
        temp.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False) + "\n")
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def make_node(rclpy, contract):
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState

    class Feedback(Node):
        def __init__(self):
            super().__init__("teacher_actual_sensor_feedback", enable_rosout=False, start_parameter_services=False)
            self.histories = {key: deque(maxlen=HISTORY_SIZE) for key in TOPICS}
            self.received, self.accepted, self.invalid, self.invalid_reasons = Counter(), Counter(), Counter(), Counter()
            self.sequence = 0
            self.started = time.monotonic()
            self.last_contract_check = -math.inf
            self.last_graph = {}
            self.alive = True
            self.script_sha256 = digest(__file__)
            self.previous_source_status = {}
            self.events = (contract.run / "sensor_feedback_events.jsonl").open("x", buffering=1)
            self.samples = (contract.run / "sensor_feedback_samples.jsonl").open("x", buffering=1)
            self.subscriptions_owned = [
                self.create_subscription(Imu, TOPICS["imu"], lambda message: self.ingest("imu", message), qos_profile_sensor_data),
                self.create_subscription(JointState, TOPICS["joints"], lambda message: self.ingest("joints", message), qos_profile_sensor_data),
            ]
            if [sub.topic_name for sub in self.subscriptions_owned] != list(TOPICS.values()):
                raise ValueError("actual_sensor_topic_remapping_forbidden")
            if self.application_publishers():
                raise ValueError("sensor_feedback_has_unexpected_application_publisher")
            self.timer = self.create_timer(WRITE_PERIOD_S, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.event("started_missing_until_actual_messages", topics=TOPICS, contract=contract.description)
            self.tick()

        def application_publishers(self):
            return [publisher.topic_name for publisher in self.publishers if publisher.topic_name not in ["/rosout", "/parameter_events"]]

        def event(self, kind, **fields):
            self.events.write(json.dumps({"kind": kind, "monotonic_wall": time.monotonic(), **fields}, allow_nan=False) + "\n")

        def ingest(self, key, message):
            received = time.monotonic()
            self.received[key] += 1
            try:
                stamp = stamp_ns(message)
                if self.histories[key] and stamp <= self.histories[key][-1]["stamp_ns"]:
                    raise ValueError("source_stamp_repeated_or_reversed")
                values = contract.imu_values(message) if key == "imu" else contract.joint_values(message)
                sample = {"stamp_ns": stamp, "t": stamp*1e-9, "received_monotonic_wall": received,
                          "source_topic": TOPICS[key], "source": "actual_ros_subscription_callback",
                          "frame": message.header.frame_id, "sequence": self.accepted[key], **values}
                self.histories[key].append(sample)
                self.accepted[key] += 1
                self.samples.write(json.dumps({"stream": key, **sample}, allow_nan=False) + "\n")
            except (ValueError, TypeError, IndexError, AttributeError) as error:
                self.invalid[key] += 1
                reason = str(error)
                self.invalid_reasons[f"{key}:{reason}"] += 1
                self.event("invalid_actual_message", stream=key, reason=reason, frame=getattr(message.header, "frame_id", None))

        def tick(self):
            now = time.monotonic()
            if self.alive and now - self.last_contract_check >= 1:
                contract.assert_unchanged()
                self.last_graph = {key: [{"node_name": p.node_name, "node_namespace": p.node_namespace, "topic_type": p.topic_type}
                                        for p in self.get_publishers_info_by_topic(topic)] for key, topic in TOPICS.items()}
                self.last_contract_check = now
            statuses = {}
            for key, history in self.histories.items():
                age = now - history[-1]["received_monotonic_wall"] if history else None
                statuses[key] = {"status": "missing" if age is None else "stale_wall" if age > WALL_MAX_AGE_S else "actual_history_available",
                                 "wall_age_s": age, "received": self.received[key], "accepted": self.accepted[key], "invalid": self.invalid[key]}
                if self.previous_source_status.get(key) != statuses[key]["status"]:
                    self.event("source_status_changed", stream=key, **statuses[key])
                    self.previous_source_status[key] = statuses[key]["status"]
            self.sequence += 1
            payload = {
                "schema_version": 1, "run": str(contract.run), "broker_pid": os.getpid(), "broker_alive": self.alive,
                "broker_sequence": self.sequence, "broker_monotonic_wall": now, "broker_elapsed_wall_s": now-self.started,
                "topics": TOPICS, "history_maxlen": HISTORY_SIZE, "source_status": statuses,
                "exported_history_maxlen": EXPORTED_HISTORY_SIZE, "write_period_s": WRITE_PERIOD_S,
                "exported_history_counts": {key: min(len(history), EXPORTED_HISTORY_SIZE) for key, history in self.histories.items()},
                "histories": {key: list(history)[-EXPORTED_HISTORY_SIZE:] for key, history in self.histories.items()},
                "raw_history_scope": "Internal buffers retain80 per stream; complete accepted messages remain in sensor_feedback_samples.jsonl. Only atomic live payload is bounded to the latest16.",
                "invalid_reason_counts": dict(self.invalid_reasons), "contract": contract.description,
                "publisher_graph": self.last_graph, "publisher_graph_monotonic_wall": self.last_contract_check,
                "application_publishers": self.application_publishers(), "script_sha256": self.script_sha256,
                "replacement_scope": "Only actual IMU body gyro/gravity and named q/qd, 30 dimensions; no actor, torque, velocity, terrain scan or SLAM construction",
                "worker_required_selection": {"causal_sensor_stamp": "<= native effective physical world time, world_sim_time-.005",
                                              "max_source_age_sim_s": .025, "max_source_age_wall_s": WALL_MAX_AGE_S,
                                              "initialization": "Worker uses privileged input until t<3; broker never makes a policy choice",
                                              "after_activation": "Worker must fail to damping on missing/invalid/stale data, no truth fallback"},
                "navigation_truth_used": False, "hardware_estimator_validated": False,
            }
            atomic_json(contract.run / "sensor_feedback.json", payload)

        def finish(self):
            self.alive = False
            # Contract/frame/receive errors are never disguised as a final ready source.
            try:
                self.tick()
                self.event("broker_ended", source_status={key: {"accepted": self.accepted[key], "invalid": self.invalid[key]} for key in TOPICS})
            finally:
                self.events.close()
                self.samples.close()

    return Feedback()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--prepare", action="store_true", help="Only verify archived run contracts; no ROS, no output writes")
    args, ros_args = parser.parse_known_args()
    contract = FeedbackContract(args.run)
    if args.prepare:
        print(json.dumps({"status": "offline_contract_checked_only", "contract": contract.description, "topics": TOPICS,
                          "histories_api": {"imu": ["stamp_ns", "received_monotonic_wall", "gyro_body", "gravity_body"],
                                            "joints": ["stamp_ns", "received_monotonic_wall", "q", "qd", "ordered_names"]}}, indent=2))
        return
    import rclpy
    from rclpy.executors import ExternalShutdownException
    # This lock prevents concurrent brokers from writing the same run's source file.
    with (contract.run / "sensor_feedback.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rclpy.init(args=ros_args)
        node = None
        try:
            node = make_node(rclpy, contract)
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            if node is not None:
                try:
                    node.finish()
                finally:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
