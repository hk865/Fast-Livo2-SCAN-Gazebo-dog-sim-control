"""Bounded, CPU-only frozen-submap proposal contract; no matcher or runtime.

Transforms use T_target_source. The independent matcher supplies T_map_body;
the core computes T_map_odom = T_map_body @ inverse(T_odom_body) at ONE source
stamp. Nothing here publishes a TF, changes an EKF, resets z, differentiates a
correction, changes a region contract, or permits control.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
from datetime import datetime
import hashlib
import math
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence

import numpy as np


MAP_ORDER = ("F1", "ramp12_transition", "F2", "ramp23_transition", "F3")
MAX_MAP_BYTES = 128 * 1024 * 1024
SCHEMA = "submap_shadow_proposal/v1"


class ContractError(ValueError):
    """An unknown, malformed, unbound or untrusted input is not evidence."""


def _integer(value: Any, name: str, *, positive: bool = False) -> int:
    if type(value) is not int or value < int(positive):
        raise ContractError(f"{name}: expected exact {'positive' if positive else 'nonnegative'} integer")
    return value


def _text(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 256:
        raise ContractError(f"{name}: bounded nonempty string required")
    return value


def _sha(value: Any, name: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ContractError(f"{name}: lowercase SHA256 required")
    return value


def _number(value: Any, name: str, *, minimum: float = 0.) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ContractError(f"{name}: finite numeric evidence required")
    result = float(value)
    if not math.isfinite(result) or result < minimum:
        raise ContractError(f"{name}: finite value >= {minimum} required")
    return result


def _array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    if np.shape(value) != shape:
        raise ContractError(f"{name}: expected shape {shape}")
    result = np.array(value, dtype=np.float64, copy=True)
    if not np.isfinite(result).all():
        raise ContractError(f"{name}: nonfinite value")
    result.flags.writeable = False
    return result


def _se3(value: Any, name: str) -> np.ndarray:
    result = _array(value, (4, 4), name)
    r = result[:3, :3]
    if (not np.allclose(result[3], (0., 0., 0., 1.), atol=1e-10, rtol=0.)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-8, rtol=0.)
            or abs(np.linalg.det(r) - 1.) > 1e-8):
        raise ContractError(f"{name}: finite proper SE(3) required")
    return result


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return float(math.acos(float(np.clip((np.trace(a.T @ b) - 1.) / 2., -1., 1.))))


def _direction_angle(a: np.ndarray, b: np.ndarray) -> float:
    return float(math.acos(float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1., 1.))))


def _transform_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), _angle(a[:3, :3], b[:3, :3])


@dataclass(frozen=True)
class FrozenMap:
    """File hash verified locally; provenance/geometry attestations still need review.

    source_kind must describe a separately captured sensor map, never the
    current frontend map or Gazebo truth. A valid SHA authenticates bytes, NOT
    physical accuracy or probabilistic independence.
    """
    map_id: str
    frame_id: str
    revision: int
    content_sha256: str
    byte_count: int
    source_capture_id: str
    source_session_id: str
    source_start_ns: int
    source_end_ns: int
    source_time_domain: str
    frozen_at_utc: str
    calibration_sha256: str
    preprocessing_sha256: str
    validation_evidence_sha256: str
    independence_evidence_sha256: str
    gravity_map: np.ndarray
    gravity_evidence_sha256: str
    _content_verified: bool = field(default=False, init=False, repr=False)

    @classmethod
    def load(cls, path: str | Path, manifest: Mapping[str, Any], *, maximum_bytes: int = MAX_MAP_BYTES) -> "FrozenMap":
        _integer(maximum_bytes, "maximum map bytes", positive=True)
        if maximum_bytes > MAX_MAP_BYTES:
            raise ContractError("map budget may not exceed 128 MiB")
        if not isinstance(manifest, Mapping) or manifest.get("schema") != "frozen_sensor_submap/v1":
            raise ContractError("unknown map manifest schema")
        m = dict(manifest)
        if m.get("source_kind") != "independent_sensor_capture" or m.get("navigation_truth_used") is not False:
            raise ContractError("map must exclude truth/current-frontend geometry")
        if m.get("geometry_reviewed") is not True or m.get("independence_reviewed") is not True:
            raise ContractError("unknown map geometry or independence review")
        if m.get("map_id") not in MAP_ORDER:
            raise ContractError("unknown map id")
        _text(m.get("frame_id"), "map frame")
        _integer(m.get("revision"), "map revision", positive=True)
        for key in ("content_sha256", "calibration_sha256", "preprocessing_sha256",
                    "validation_evidence_sha256", "independence_evidence_sha256", "gravity_evidence_sha256"):
            _sha(m.get(key), key)
        for key in ("source_capture_id", "source_session_id", "source_time_domain"):
            _text(m.get(key), key)
        start = _integer(m.get("source_start_ns"), "map start time", positive=True)
        end = _integer(m.get("source_end_ns"), "map end time", positive=True)
        if end < start:
            raise ContractError("map source time range reversed")
        try:
            freeze = datetime.fromisoformat(_text(m.get("frozen_at_utc"), "freeze UTC").replace("Z", "+00:00"))
        except ValueError as exc:
            raise ContractError("map freeze time malformed") from exc
        if freeze.utcoffset() is None or freeze.utcoffset().total_seconds() != 0:
            raise ContractError("map freeze time must explicitly use UTC")
        gravity = _array(m.get("gravity_map"), (3,), "frozen map gravity")
        if not 8. <= np.linalg.norm(gravity) <= 12.:
            raise ContractError("map gravity needs validated magnitude and direction")
        file = Path(path)
        before = file.stat()
        if not file.is_file() or not 0 < before.st_size <= maximum_bytes:
            raise ContractError("map file empty or exceeds bounded budget")
        h, count = hashlib.sha256(), 0
        with file.open("rb") as stream:
            while block := stream.read(1024 * 1024):
                count += len(block)
                if count > maximum_bytes:
                    raise ContractError("map grew beyond bounded budget")
                h.update(block)
        after = file.stat()
        if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
            raise ContractError("map changed during verification")
        if count != before.st_size or h.hexdigest() != m["content_sha256"]:
            raise ContractError("frozen map content hash mismatch")
        result = cls(m["map_id"], m["frame_id"], m["revision"], m["content_sha256"], count,
                   m["source_capture_id"], m["source_session_id"], start, end, m["source_time_domain"],
                   m["frozen_at_utc"], m["calibration_sha256"], m["preprocessing_sha256"],
                   m["validation_evidence_sha256"], m["independence_evidence_sha256"], gravity,
                   m["gravity_evidence_sha256"])
        object.__setattr__(result, "_content_verified", True)
        return result


@dataclass(frozen=True)
class Overlap:
    """Adjacent frozen-map relation and shared volume; never inferred from a label.

    left_from_right transforms right-map coordinates to left-map coordinates.
    The volume is expressed in left-map coordinates, verified by an external
    geometry review identified by evidence_sha256. Its construction is not a
    registration algorithm and the hash alone cannot validate that review.
    """
    left: str
    right: str
    left_from_right: np.ndarray
    left_min: np.ndarray
    left_max: np.ndarray
    left_map_sha256: str
    right_map_sha256: str
    evidence_sha256: str
    navigation_truth_used: bool
    independently_reviewed: bool

    def __post_init__(self):
        if (self.left, self.right) not in tuple(zip(MAP_ORDER[:-1], MAP_ORDER[1:])):
            raise ContractError("only adjacent platform/transition-map overlaps are permitted")
        for key in ("left_map_sha256", "right_map_sha256", "evidence_sha256"):
            _sha(getattr(self, key), key)
        if self.navigation_truth_used is not False or self.independently_reviewed is not True:
            raise ContractError("unreviewed or truth-derived overlap")
        object.__setattr__(self, "left_from_right", _se3(self.left_from_right, "overlap transform"))
        object.__setattr__(self, "left_min", _array(self.left_min, (3,), "overlap minimum"))
        object.__setattr__(self, "left_max", _array(self.left_max, (3,), "overlap maximum"))
        if not np.all(self.left_max > self.left_min):
            raise ContractError("overlap must have positive extent in all axes")


@dataclass(frozen=True)
class MatchQuery:
    session_id: str
    odom_revision: int
    source_ns: int
    received_wall_ns: int
    odom_frame: str
    body_frame: str
    odom_from_body: np.ndarray
    query_payload_sha256: str
    calibration_sha256: str
    gravity_odom: np.ndarray
    gravity_evidence_sha256: str
    geometry_source: str = "same_source_local_sensor_geometry"
    gravity_source: str = "actual_calibrated_imu_direction"
    navigation_truth_used: bool = False

    def __post_init__(self):
        for key in ("session_id", "odom_frame", "body_frame"):
            _text(getattr(self, key), key)
        for key in ("odom_revision", "received_wall_ns"):
            _integer(getattr(self, key), key)
        _integer(self.source_ns, "query source", positive=True)
        for key in ("query_payload_sha256", "calibration_sha256", "gravity_evidence_sha256"):
            _sha(getattr(self, key), key)
        object.__setattr__(self, "odom_from_body", _se3(self.odom_from_body, "same-source odom pose"))
        object.__setattr__(self, "gravity_odom", _array(self.gravity_odom, (3,), "query gravity"))
        if not 8. <= np.linalg.norm(self.gravity_odom) <= 12.:
            raise ContractError("query gravity magnitude invalid")
        if (self.geometry_source != "same_source_local_sensor_geometry"
                or self.gravity_source != "actual_calibrated_imu_direction" or self.navigation_truth_used is not False):
            raise ContractError("only same-source sensor geometry and calibrated IMU evidence allowed")


@dataclass(frozen=True)
class MatchCandidate:
    session_id: str
    odom_revision: int
    matcher_session_id: str
    matcher_revision: int
    matcher_source_sha256: str
    source_ns: int
    received_wall_ns: int
    map_id: str
    map_revision: int
    map_frame: str
    body_frame: str
    map_sha256: str
    query_payload_sha256: str
    calibration_sha256: str
    map_from_body: np.ndarray
    evidence: Mapping[str, Any]
    navigation_truth_used: bool = False
    schema: str = "independent_submap_match/v1"

    def __post_init__(self):
        object.__setattr__(self, "map_from_body", _se3(self.map_from_body, "matched body pose"))
        # Evidence is copied/validated inside offer, never used as a live object.


class IndependentMatcher(Protocol):
    """Future adapter: only match these verified frozen maps against this query.

    The implementation must retain query payload/source pairing and produce
    distinct physical-pose hypotheses, including lookalike floors. Coincident
    hypotheses from overlapping maps can only be merged under reviewed edges.
    This protocol has NO implemented matcher or cloud ingestion in V33.
    """
    def match(self, query: MatchQuery, maps: Mapping[str, FrozenMap]) -> Sequence[MatchCandidate]: ...


def probe_existing_sensor_sources(paths: Mapping[str, str | Path], hashes: Mapping[str, str], *,
                                  source_ns: int, seed_z_offsets: Sequence[float],
                                  prior_information: Any = None) -> dict:
    """Read-only, bounded source feasibility/ICP probe, NOT IndependentMatcher.

    Accepts the existing RGB binary PCD/save metadata and normalized registered
    NPY/archive/exact body-odom records. No SDF, truth alignment, floor height,
    covariance, or mount transform is used. The source pose is an initialization
    only; prior information is refused. This diagnostic never emits candidates.
    Seed offsets are explicit caller choices, not a complete floor search.
    """
    import io
    import json
    import time
    from scipy.spatial import cKDTree

    keys = {"pcd", "map_metadata", "query", "archive", "pose"}
    if set(paths) != keys or set(hashes) != keys:
        raise ContractError("five_explicit_source_paths_and_hashes_required")
    if prior_information is not None:
        raise ContractError("prior_information_not_permitted_in_geometry_probe")
    _integer(source_ns, "probe source", positive=True)
    if not 2 <= len(seed_z_offsets) <= 8:
        raise ContractError("probe_requires_two_to_eight_explicit_seeds")
    offsets = []
    for value in seed_z_offsets:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > 10.:
            raise ContractError("bounded_finite_seed_offsets_required")
        offsets.append(float(value))
    if len(set(offsets)) != len(offsets):
        raise ContractError("duplicate_probe_seeds")
    blobs, bindings = {}, {}
    for key in sorted(keys):
        path = Path(paths[key]); expected = _sha(hashes[key], key)
        with path.open("rb") as stream:
            blob = stream.read(16 * 1024 * 1024 + 1)
        if not 0 < len(blob) <= 16 * 1024 * 1024 or hashlib.sha256(blob).hexdigest() != expected:
            raise ContractError("probe_source_size_or_hash_mismatch:" + key)
        blobs[key] = blob
        bindings[key] = {"path": str(path.resolve()), "bytes": len(blob), "sha256": expected}

    def exact_record(key):
        found = []
        for line in blobs[key].splitlines():
            if len(line) > 64 * 1024:
                raise ContractError("oversized_source_metadata_record")
            record = json.loads(line)
            if record.get("stamp_ns") == source_ns:
                found.append(record)
        if len(found) != 1:
            raise ContractError("one_exact_integer_source_record_required:" + key)
        return found[0]

    archive, odom = exact_record("archive"), exact_record("pose")
    if (archive.get("schema") != "actual_registered_cloud_xyz_archive/v1"
            or archive.get("navigation_ground_truth_used") is not False
            or odom.get("navigation_ground_truth_used") is not False
            or archive.get("frame_id") != "camera_init" or odom.get("frame_id") != "camera_init"
            or odom.get("child_frame_id") != "demo_slam_body"
            or odom.get("source_topic") != "/demo/slam/body_odom"
            or type(archive.get("stamp_ns")) is not int or type(odom.get("stamp_ns")) is not int
            or archive.get("filtering_body_stamp_ns") != source_ns
            or Path(archive.get("array_file", "")).name != Path(paths["query"]).name):
        raise ContractError("query_source_frame_reference_or_truth_mismatch")
    transform = np.eye(4)
    transform[:3, :3] = _array(archive.get("filtering_body_rotation"), (3, 3), "source rotation")
    transform[:3, 3] = _array(archive.get("filtering_body_pose"), (3,), "source position")
    transform = _se3(transform, "registered_from_body")
    x, y, z, w = _array(odom.get("quaternion_xyzw"), (4,), "source quaternion")
    if abs(x*x + y*y + z*z + w*w - 1.) > 1e-8:
        raise ContractError("source_quaternion_not_unit")
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    if (not np.allclose(rotation, transform[:3, :3], atol=1e-10, rtol=0.)
            or not np.allclose(_array(odom.get("position"), (3,), "odom position"), transform[:3, 3], atol=1e-10, rtol=0.)):
        raise ContractError("same_source_body_pose_disagrees_with_cloud_archive")
    stream = io.BytesIO(blobs["query"])
    version = np.lib.format.read_magic(stream)
    if version not in ((1, 0), (2, 0)):
        raise ContractError("unsupported_normalized_query_format")
    reader = np.lib.format.read_array_header_1_0 if version == (1, 0) else np.lib.format.read_array_header_2_0
    shape, fortran, dtype = reader(stream, max_header_size=1024)
    if (dtype != np.dtype("<f8") or fortran or len(shape) != 2 or shape[1] != 3
            or not 100 <= shape[0] <= 250000 or list(shape) != archive.get("shape")
            or stream.tell() + shape[0] * 24 != len(blobs["query"])
            or hashes["query"] != archive.get("array_sha256")):
        raise ContractError("query_layout_or_archive_hash_mismatch")
    xyz = np.frombuffer(blobs["query"], dtype="<f8", offset=stream.tell()).reshape(shape)
    if (not np.isfinite(xyz).all() or hashlib.sha256(xyz.tobytes()).hexdigest()
            != archive.get("filtered_xyz_float64_sha256")):
        raise ContractError("normalized_query_xyz_hash_mismatch")
    selected = xyz[::math.ceil(len(xyz)/4000)]
    local = (selected - transform[:3, 3]) @ transform[:3, :3]

    metadata = json.loads(blobs["map_metadata"]); saved = metadata.get("save", {})
    if (metadata.get("frame_id") != "camera_init" or metadata.get("ground_truth_used") is not False
            or metadata.get("reference_map_loaded") is not False or saved.get("complete") is not True
            or saved.get("filename") != Path(paths["pcd"]).name
            or metadata.get("format", {}).get("xyz") != "little-endian float32 at offsets 0,4,8"):
        raise ContractError("unbound_saved_sensor_map_metadata")
    blob = blobs["pcd"]; end = blob.find(b"DATA binary\n", 0, 16384)
    if end < 0:
        raise ContractError("unsupported_sensor_pcd_header")
    offset = end + len(b"DATA binary\n"); header = blob[:offset].decode("ascii")
    required = ("FIELDS x y z rgb\n", "SIZE 4 4 4 4\n", "TYPE F F F U\n", "COUNT 1 1 1 1\n", "HEIGHT 1\n")
    count = _integer(saved.get("point_count"), "saved points", positive=True)
    if (any(line not in header for line in required) or f"POINTS {count}\n" not in header
            or f"WIDTH {count}\n" not in header or offset + count*16 != len(blob)):
        raise ContractError("saved_pcd_layout_or_snapshot_count_mismatch")
    packed = np.frombuffer(blob, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")], offset=offset)
    indices = np.arange(0, count, math.ceil(count/32000))
    target = np.column_stack([packed[k][indices] for k in ("x", "y", "z")]).astype(np.float64)
    if not np.isfinite(target).all():
        raise ContractError("nonfinite_sampled_map_geometry")
    tree = cKDTree(target); started = time.monotonic(); hypotheses = []
    for dz in offsets:
        current = np.array(transform, copy=True); current[2, 3] += dz; converged = False
        for iteration in range(40):
            moved = local @ current[:3, :3].T + current[:3, 3]
            distances, matches = tree.query(moved, workers=1); valid = distances <= .5
            if valid.sum() < 80 or time.monotonic() - started > 15.:
                break
            a, b = moved[valid], target[matches[valid]]; ac, bc = a.mean(0), b.mean(0)
            u, _, vt = np.linalg.svd((a-ac).T @ (b-bc)); r = vt.T @ u.T
            if np.linalg.det(r) < 0:
                vt[-1] *= -1.; r = vt.T @ u.T
            step = np.eye(4); step[:3, :3] = r; step[:3, 3] = bc-r@ac; current = step@current
            if np.linalg.norm(step[:3, 3]) < 1e-5 and np.linalg.norm(r-np.eye(3)) < 1e-5:
                converged = True; break
        moved = local @ current[:3, :3].T + current[:3, 3]
        distances, matches = tree.query(moved, workers=1); valid = distances <= .5
        _, backward = cKDTree(moved).query(target[matches[valid]], workers=1)
        hypotheses.append({"seed_z_offset_m": dz, "iterations": iteration+1, "converged": converged,
            "geometry_capped_all_query_rms_m": float(np.sqrt(np.mean(np.minimum(distances, .5)**2))),
            "inlier_rms_m": float(np.sqrt(np.mean(distances[valid]**2))) if valid.any() else None,
            "all_query_p95_m": float(np.quantile(distances, .95)), "within_0p5m_fraction": float(valid.mean()),
            "mutual_correspondence_fraction": float(np.mean(backward == np.flatnonzero(valid))) if valid.any() else 0.,
            "unique_target_fraction": len(np.unique(matches[valid]))/int(valid.sum()) if valid.any() else 0.,
            "map_from_body": current.tolist()})
        if time.monotonic() - started > 15.:
            break
    return {"schema": "existing_sensor_source_cpu_feasibility_probe/v1", "source_bindings": bindings,
        "query_source_ns": source_ns, "same_integer_source_pose_verified": True,
        "sample_local_roundtrip_max_m": float(np.max(np.abs(local@transform[:3, :3].T+transform[:3, 3]-selected))),
        "pcd_points": count, "pcd_saved_source_s": saved.get("last_observation_stamp"),
        "online_revision_not_used": metadata.get("revision"), "online_point_count_not_used": metadata.get("point_count"),
        "sampled_map_points": len(target), "sampled_query_points": len(local),
        "sampling": "deterministic_stride_existing_points_no_centroids", "duration_wall_s": time.monotonic()-started,
        "budget_exhausted": len(hypotheses) != len(offsets) or time.monotonic()-started > 15.,
        "objective": "point_to_point_geometry_only_initialization_probe", "prior_information_added": False,
        "seed_provenance": "same_source_frontend_pose_plus_explicit_offsets_not_truth", "hypotheses": hypotheses,
        "search_complete": False, "independent_matcher_implemented": False, "shadow_candidate_produced": False,
        "runtime_enabled": False, "control_allowed": False, "original_ros_payload_verified": False,
        "blockers": ["five_reviewed_maps_and_four_overlaps_missing", "global_lookalike_floor_search_missing",
            "geometry_independence_calibration_and_gravity_reviews_missing", "normal_information_and_association_review_missing",
            "normalized_xyz_is_not_original_ros_payload", "runtime_adapter_and_revision_ack_unverified"]}


@dataclass(frozen=True)
class ShadowPolicy:
    """Conservative prototype gates, NOT calibrated production thresholds."""
    ttl_ns: int = 300_000_000
    maximum_gap_ns: int = 150_000_000
    minimum_span_ns: int = 500_000_000
    minimum_frames: int = 5
    maximum_frames: int = 32
    minimum_inliers: int = 100
    minimum_inlier_fraction: float = .70
    minimum_overlap_fraction: float = .50
    minimum_occupied_cells: int = 8
    minimum_normal_bins: int = 3
    maximum_rms_m: float = .035
    maximum_p95_m: float = .08
    minimum_unique_score_gap_fraction: float = .20
    minimum_information_eigen_fraction: float = .001
    maximum_gravity_angle_rad: float = math.radians(3.)
    maximum_temporal_translation_m: float = .04
    maximum_temporal_rotation_rad: float = math.radians(2.)
    maximum_overlap_translation_m: float = .05
    maximum_overlap_rotation_rad: float = math.radians(2.)
    maximum_proposal_innovation_m: float = .50
    maximum_proposal_innovation_rad: float = math.radians(30.)

    def __post_init__(self):
        for key in ("ttl_ns", "maximum_gap_ns", "minimum_span_ns", "minimum_frames",
                    "maximum_frames", "minimum_inliers", "minimum_occupied_cells", "minimum_normal_bins"):
            _integer(getattr(self, key), key, positive=True)
        for key in self.__dataclass_fields__:
            if key not in ("ttl_ns", "maximum_gap_ns", "minimum_span_ns", "minimum_frames",
                           "maximum_frames", "minimum_inliers", "minimum_occupied_cells", "minimum_normal_bins"):
                _number(getattr(self, key), key, minimum=np.finfo(float).eps)
        if not (self.maximum_gap_ns <= self.ttl_ns and 2 <= self.minimum_frames <= self.maximum_frames <= 64):
            raise ContractError("invalid bounded temporal policy")
        if any(getattr(self, k) > 1. for k in ("minimum_inlier_fraction", "minimum_overlap_fraction",
                  "minimum_unique_score_gap_fraction", "minimum_information_eigen_fraction")):
            raise ContractError("fraction outside [0,1]")


@dataclass(frozen=True)
class _Accepted:
    query: MatchQuery
    candidate: MatchCandidate
    map_from_odom: np.ndarray
    canonical_from_odom: np.ndarray
    oldest_receipt_wall_ns: int
    paired_overlap: bool


class SubmapShadow:
    """Serialized, single-session gate with only diagnostic/proposal outputs.

    active_shadow_map is a reference for this graph's proposal sequence, NOT
    the active runtime map. select_shadow_reference only advances that sequence.
    All five maps and all four overlaps must be present and hash-bound.
    """
    def __init__(self, *, session_id: str, odom_revision: int, matcher_session_id: str,
                 matcher_revision: int, matcher_source_sha256: str, calibration_sha256: str,
                 odom_frame: str, body_frame: str, maps: Sequence[FrozenMap], overlaps: Sequence[Overlap],
                 policy: ShadowPolicy | None = None):
        self.session_id = _text(session_id, "runtime session")
        self.odom_revision = _integer(odom_revision, "odom revision")
        self.matcher_session_id = _text(matcher_session_id, "matcher session")
        self.matcher_revision = _integer(matcher_revision, "matcher revision")
        self.matcher_source_sha256 = _sha(matcher_source_sha256, "matcher source")
        self.calibration_sha256 = _sha(calibration_sha256, "runtime calibration")
        self.odom_frame, self.body_frame = _text(odom_frame, "odom frame"), _text(body_frame, "body frame")
        if policy is not None and not isinstance(policy, ShadowPolicy):
            raise ContractError("unknown shadow policy type")
        self.policy = policy or ShadowPolicy()
        if not all(isinstance(m, FrozenMap) and m._content_verified for m in maps) or len(maps) != len(MAP_ORDER):
            raise ContractError("all five verified frozen maps required")
        self.maps = {m.map_id: m for m in maps}
        if set(self.maps) != set(MAP_ORDER) or len({m.frame_id for m in maps}) != len(maps):
            raise ContractError("map IDs/frames incomplete or duplicated")
        if sum(m.byte_count for m in maps) > MAX_MAP_BYTES or any(m.source_session_id == session_id for m in maps):
            raise ContractError("map budget exceeded or maps reused current-session frontend")
        if len(overlaps) != len(MAP_ORDER) - 1 or not all(isinstance(e, Overlap) for e in overlaps):
            raise ContractError("all four reviewed overlaps required")
        self.edges = {(e.left, e.right): e for e in overlaps}
        if set(self.edges) != set(zip(MAP_ORDER[:-1], MAP_ORDER[1:])):
            raise ContractError("missing or duplicate overlap")
        self.canonical = {MAP_ORDER[0]: np.eye(4)}
        for left, right in zip(MAP_ORDER[:-1], MAP_ORDER[1:]):
            edge = self.edges[left, right]
            if (edge.left_map_sha256, edge.right_map_sha256) != (
                    self.maps[left].content_sha256, self.maps[right].content_sha256):
                raise ContractError("overlap bound to different frozen maps")
            self.canonical[right] = self.canonical[left] @ edge.left_from_right
        self.buffers = {name: deque(maxlen=self.policy.maximum_frames) for name in MAP_ORDER}
        self.latest = {}
        self.last_source = {name: 0 for name in MAP_ORDER}
        self.last_clock_source = self.last_clock_wall = None
        self.last_selected = None
        self.active_shadow_map = MAP_ORDER[0]
        self.proposal = None
        self.generation = 0
        self.state, self.reason = "NO_PROPOSAL", "awaiting_independent_matching_evidence"
        self.events = deque(maxlen=64)

    def _invalidate(self, state: str, reason: str, *, target: str | None = None):
        reason = str(reason)[:256]
        self.proposal = None
        self.state, self.reason = state, reason
        if target in self.buffers:
            self.buffers[target].clear()
            self.latest.pop(target, None)
        else:
            for cache in self.buffers.values():
                cache.clear()
            self.latest.clear()
        self.events.append({"state": state, "reason": reason, "target": target})

    def _clock(self, now_source_ns: int, now_wall_ns: int):
        _integer(now_source_ns, "source clock", positive=True)
        _integer(now_wall_ns, "monotonic wall clock")
        if ((self.last_clock_source is not None and now_source_ns < self.last_clock_source)
                or (self.last_clock_wall is not None and now_wall_ns < self.last_clock_wall)):
            self._invalidate("FAULT", "clock_rollback_new_isolated_session_required")
            raise ContractError(self.reason)
        self.last_clock_source, self.last_clock_wall = now_source_ns, now_wall_ns

    def _fresh(self, source_ns: int, received_wall_ns: int, now_source_ns: int, now_wall_ns: int):
        if not (0 <= now_source_ns - source_ns <= self.policy.ttl_ns
                and 0 <= now_wall_ns - received_wall_ns <= self.policy.ttl_ns):
            raise ContractError("stale_or_future_source_or_wall_receipt")

    def _identity(self, q: MatchQuery, c: MatchCandidate):
        for key in ("odom_revision", "matcher_revision", "map_revision"):
            _integer(getattr(c, key), key)
        if (q.session_id != self.session_id or c.session_id != self.session_id
                or q.odom_revision != self.odom_revision or c.odom_revision != self.odom_revision
                or c.matcher_session_id != self.matcher_session_id or c.matcher_revision != self.matcher_revision
                or c.matcher_source_sha256 != self.matcher_source_sha256):
            self._invalidate("FAULT", "session_revision_or_matcher_identity_changed_new_session_required")
            raise ContractError(self.reason)
        if c.schema != "independent_submap_match/v1" or c.navigation_truth_used is not False:
            raise ContractError("truth_or_unknown_candidate_schema")
        _integer(c.source_ns, "candidate source", positive=True)
        _integer(c.received_wall_ns, "matcher receipt")
        if c.source_ns != q.source_ns or c.query_payload_sha256 != q.query_payload_sha256:
            raise ContractError("candidate_query_not_same_source_and_payload")
        if (q.odom_frame, q.body_frame, c.body_frame) != (self.odom_frame, self.body_frame, self.body_frame):
            raise ContractError("odom_or_body_frame_mismatch")
        if c.calibration_sha256 != self.calibration_sha256 or q.calibration_sha256 != self.calibration_sha256:
            raise ContractError("current_sensor_calibration_mismatch")
        if c.map_id not in self.maps:
            raise ContractError("unknown_candidate_map")
        m = self.maps[c.map_id]
        if (c.map_revision, c.map_frame, c.map_sha256) != (m.revision, m.frame_id, m.content_sha256):
            raise ContractError("candidate_not_bound_to_verified_frozen_map")
        if abs(MAP_ORDER.index(c.map_id) - MAP_ORDER.index(self.active_shadow_map)) > 1:
            raise ContractError("nonadjacent_map_requires_overlap_chain")

    def _quality(self, q: MatchQuery, c: MatchCandidate, correction: np.ndarray) -> dict:
        if not isinstance(c.evidence, Mapping):
            raise ContractError("unknown_matching_evidence")
        fields = {"schema", "matcher_kind", "converged", "search_complete", "independent_association_reviewed",
                  "search_domain_sha256", "association_evidence_sha256", "tested_map_ids", "distinct_hypotheses",
                  "best_score_m", "second_score_m", "rms_m", "p95_m", "inliers", "occupied_cells", "normal_bins",
                  "support_span_m", "inlier_fraction", "overlap_fraction", "information_model",
                  "parameter_scales_m_rad", "normalized_information"}
        if set(c.evidence) != fields:
            raise ContractError("missing_or_unknown_matching_evidence_fields")
        e = dict(c.evidence)
        if (e.get("schema") != "independent_match_evidence/v1"
                or e.get("matcher_kind") != "independent_frozen_map_registration"
                or any(e.get(k) is not True for k in ("converged", "search_complete", "independent_association_reviewed"))):
            raise ContractError("missing_independent_registration_or_complete_search")
        _sha(e.get("search_domain_sha256"), "search domain")
        _sha(e.get("association_evidence_sha256"), "matching associations")
        tested = e.get("tested_map_ids")
        if (not isinstance(tested, (tuple, list)) or len(tested) != len(MAP_ORDER)
                or set(tested) != set(MAP_ORDER)):
            raise ContractError("lookalike_maps_not_all_tested")
        p = self.policy
        if _integer(e.get("distinct_hypotheses"), "distinct physical hypotheses", positive=True) < 2:
            raise ContractError("uniqueness_unmeasured")
        best, second = _number(e.get("best_score_m"), "best hypothesis RMS"), _number(e.get("second_score_m"), "second hypothesis RMS")
        if second <= 0. or best > second or (second - best) / second < p.minimum_unique_score_gap_fraction:
            raise ContractError("ambiguous_matching_hypotheses")
        rms, p95 = _number(e.get("rms_m"), "registration RMS"), _number(e.get("p95_m"), "registration p95")
        if best != rms or rms > p95 or rms > p.maximum_rms_m or p95 > p.maximum_p95_m:
            raise ContractError("residual_distribution_exceeds_shadow_policy")
        if (_integer(e.get("inliers"), "inliers", positive=True) < p.minimum_inliers
                or _integer(e.get("occupied_cells"), "occupied cells", positive=True) < p.minimum_occupied_cells
                or _integer(e.get("normal_bins"), "normal bins", positive=True) < p.minimum_normal_bins):
            raise ContractError("insufficient_spatial_or_normal_distribution")
        span = _array(e.get("support_span_m"), (3,), "support span")
        if np.any(span < (.5, .5, .2)):
            raise ContractError("insufficient_three_dimensional_support")
        for key, minimum in (("inlier_fraction", p.minimum_inlier_fraction), ("overlap_fraction", p.minimum_overlap_fraction)):
            value = _number(e.get(key), key)
            if not minimum <= value <= 1.:
                raise ContractError("insufficient_" + key)
        if e.get("information_model") != "normalized_geometry_jacobian_gram/v1" or e.get("parameter_scales_m_rad") != [1., 1.]:
            raise ContractError("unknown_information_units_or_model")
        gram = _array(e.get("normalized_information"), (6, 6), "geometry information")
        if not np.allclose(gram, gram.T, atol=1e-10, rtol=0.):
            raise ContractError("geometry_information_not_symmetric")
        values = np.linalg.eigvalsh(gram)
        if values[-1] <= 0. or values[0] / values[-1] < p.minimum_information_eigen_fraction:
            raise ContractError("unobservable_coupled_geometry")
        # Full-rank coupled geometry is required; a tiny P or large z diagonal
        # cannot substitute for this test. This Gram is NOT a calibrated P.
        gravity = correction[:3, :3] @ q.gravity_odom
        if _direction_angle(gravity, self.maps[c.map_id].gravity_map) > p.maximum_gravity_angle_rad:
            raise ContractError("gravity_attitude_inconsistent")
        # Retain scalar evidence and six-by-six information only. No cloud,
        # images, candidate object graphs or foreign mutable containers survive.
        import copy
        return copy.deepcopy(e)

    def _overlap_pair(self, target: str, sample: _Accepted, now_source_ns: int, now_wall_ns: int) -> bool:
        if target == self.active_shadow_map:
            return False
        other = self.latest.get(self.active_shadow_map)
        if other is None or (other.query.source_ns, other.query.query_payload_sha256) != (
                sample.query.source_ns, sample.query.query_payload_sha256):
            raise ContractError("awaiting_same_source_active_overlap_match")
        self._fresh(other.query.source_ns, other.oldest_receipt_wall_ns, now_source_ns, now_wall_ns)
        left, right = sorted((target, self.active_shadow_map), key=MAP_ORDER.index)
        edge = self.edges[left, right]
        l = sample if target == left else other
        r = sample if target == right else other
        right_in_left = edge.left_from_right @ r.candidate.map_from_body
        a, b = l.candidate.map_from_body, right_in_left
        for position in (a[:3, 3], b[:3, 3]):
            if np.any(position < edge.left_min) or np.any(position > edge.left_max):
                raise ContractError("candidate_outside_reviewed_overlap_volume")
        translation, rotation = _transform_error(a, b)
        if (translation > self.policy.maximum_overlap_translation_m
                or rotation > self.policy.maximum_overlap_rotation_rad):
            raise ContractError("overlap_transform_inconsistent")
        return True

    def offer(self, query: MatchQuery, candidate: MatchCandidate, *, now_source_ns: int, now_wall_ns: int) -> dict:
        """Validate one same-stamp independent match; all output remains shadow.

        For handoff, offer the active-map match BEFORE the adjacent-map match
        at each source stamp. This is explicit serialized pairing, not nearest
        time matching. Invalid target evidence clears that target's history;
        invalid current evidence also clears any pending transition histories.
        """
        target = getattr(candidate, "map_id", None)
        try:
            self._clock(now_source_ns, now_wall_ns)
            self.tick(now_source_ns=now_source_ns, now_wall_ns=now_wall_ns)
            if self.state == "FAULT":
                raise ContractError(self.reason)
            if not isinstance(query, MatchQuery) or not isinstance(candidate, MatchCandidate):
                raise ContractError("unknown_query_or_candidate_type")
            self._identity(query, candidate)
            self._fresh(query.source_ns, query.received_wall_ns, now_source_ns, now_wall_ns)
            self._fresh(candidate.source_ns, candidate.received_wall_ns, now_source_ns, now_wall_ns)
            if query.source_ns <= self.last_source[target]:
                raise ContractError("duplicate_or_out_of_order_source")
            self.last_source[target] = query.source_ns
            correction = _se3(candidate.map_from_body @ np.linalg.inv(query.odom_from_body), "map_from_odom")
            evidence = self._quality(query, candidate, correction)
            candidate = replace(candidate, evidence=evidence)
            canonical = _se3(self.canonical[target] @ correction, "canonical_from_odom")
            sample = _Accepted(query, candidate, correction, canonical,
                               min(query.received_wall_ns, candidate.received_wall_ns), False)
            paired = self._overlap_pair(target, sample, now_source_ns, now_wall_ns)
            sample = _Accepted(query, candidate, correction, canonical, sample.oldest_receipt_wall_ns, paired)
            cache = self.buffers[target]
            if cache and sample.oldest_receipt_wall_ns < cache[-1].oldest_receipt_wall_ns:
                raise ContractError("wall_receipt_order_violation")
            if cache and (query.source_ns - cache[-1].query.source_ns > self.policy.maximum_gap_ns
                    or sample.oldest_receipt_wall_ns - cache[-1].oldest_receipt_wall_ns > self.policy.maximum_gap_ns):
                cache.clear()
            if cache:
                for old in cache:
                    dt, dr = _transform_error(sample.map_from_odom, old.map_from_odom)
                    if dt > self.policy.maximum_temporal_translation_m or dr > self.policy.maximum_temporal_rotation_rad:
                        raise ContractError("multiframe_transform_inconsistent")
            if self.last_selected is not None:
                dt, dr = _transform_error(canonical, self.last_selected["canonical_from_odom"])
                if dt > self.policy.maximum_proposal_innovation_m or dr > self.policy.maximum_proposal_innovation_rad:
                    raise ContractError("proposal_innovation_exceeds_shadow_policy")
            cache.append(sample)
            self.latest[target] = sample
            self.proposal = None
            self.state, self.reason = "COLLECTING", "awaiting_continuous_multiframe_evidence"
            if (len(cache) >= self.policy.minimum_frames
                    and query.source_ns - cache[0].query.source_ns >= self.policy.minimum_span_ns
                    and sample.oldest_receipt_wall_ns - cache[0].oldest_receipt_wall_ns >= self.policy.minimum_span_ns):
                self.generation += 1
                self.proposal = self._proposal(sample, len(cache))
                self.state, self.reason = "PROPOSAL_READY", "shadow_contract_satisfied_no_runtime_adoption"
        except (ContractError, TypeError, ValueError, KeyError, np.linalg.LinAlgError) as exc:
            if self.state != "FAULT":
                # Current-map failure invalidates any transition that used it.
                self._invalidate("REJECTED", str(exc), target=None if target == self.active_shadow_map else target)
        return self.status()

    def _proposal(self, sample: _Accepted, frames: int) -> dict:
        c, q, m = sample.candidate, sample.query, self.maps[sample.candidate.map_id]
        return {"schema": SCHEMA, "session_id": self.session_id, "odom_revision": self.odom_revision,
                "matcher_session_id": self.matcher_session_id, "matcher_revision": self.matcher_revision,
                "matcher_source_sha256": self.matcher_source_sha256, "generation": self.generation,
                "source_ns": q.source_ns, "received_wall_ns": sample.oldest_receipt_wall_ns,
                "map_id": c.map_id, "map_revision": m.revision, "map_frame": m.frame_id,
                "odom_frame": self.odom_frame, "map_sha256": m.content_sha256,
                "query_payload_sha256": q.query_payload_sha256, "calibration_sha256": self.calibration_sha256,
                "map_from_odom": sample.map_from_odom.tolist(), "canonical_from_odom": sample.canonical_from_odom.tolist(),
                "evidence_frames": frames, "same_source_overlap_verified": sample.paired_overlap,
                "runtime_enabled": False, "control_allowed": False, "navigation_truth_used": False,
                "calibrated_uncertainty": False, "covariance_valid": False,
                "velocity_action": "none_never_differentiate_correction",
                "required_before_control": ["review_actual_map_and_matching_evidence", "validate_runtime_frames_and_mounts",
                    "supervisor_revision_barrier_and_owned_planner_reset_ack", "fresh_local_cloud_rebuild",
                    "same_generation_path_admission", "preserve_original_region_and_parking_contracts",
                    "bounded_closed_loop_acceptance"]}

    def select_shadow_reference(self, generation: int, *, now_source_ns: int, now_wall_ns: int) -> dict:
        """Advance ONLY this prototype's graph context, never the runtime map.

        No supervisor ACK or process reset is fabricated. This method is for
        sequential shadow evaluation of later overlaps after a proposal review.
        """
        self.tick(now_source_ns=now_source_ns, now_wall_ns=now_wall_ns)
        if self.state != "PROPOSAL_READY" or self.proposal is None or type(generation) is not int or generation != self.proposal["generation"]:
            raise ContractError("no_fresh_matching_shadow_proposal")
        selected = dict(self.proposal)
        self.last_selected = {"canonical_from_odom": _se3(selected["canonical_from_odom"], "selected correction")}
        self.active_shadow_map = selected["map_id"]
        self._invalidate("NO_PROPOSAL", "shadow_reference_selected_runtime_unchanged")
        return self.status()

    def tick(self, *, now_source_ns: int, now_wall_ns: int) -> dict:
        self._clock(now_source_ns, now_wall_ns)
        if self.state == "FAULT":
            return self.status()
        for name, sample in list(self.latest.items()):
            try:
                self._fresh(sample.query.source_ns, sample.oldest_receipt_wall_ns, now_source_ns, now_wall_ns)
            except ContractError:
                self.buffers[name].clear()
                self.latest.pop(name, None)
                if self.proposal is not None and self.proposal["map_id"] == name:
                    self._invalidate("EXPIRED", "proposal_source_or_wall_receipt_expired", target=name)
                elif name == self.active_shadow_map:
                    for other in self.buffers:
                        if other != name:
                            self.buffers[other].clear()
                            self.latest.pop(other, None)
                    self.proposal = None
                    self.state, self.reason = "EXPIRED", "active_overlap_evidence_expired"
        return self.status()

    def status(self) -> dict:
        # Deep scalar/list copy makes callers unable to mutate a pending token.
        import copy
        return {"schema": "submap_shadow_status/v1", "state": self.state, "reason": self.reason,
                "runtime_enabled": False, "control_allowed": False, "matcher_implemented": False,
                "actual_map_accuracy_verified": False, "actual_localization_verified": False,
                "calibrated_uncertainty": False, "active_shadow_map": self.active_shadow_map,
                "generation": self.generation, "proposal": copy.deepcopy(self.proposal),
                "buffer_counts": {name: len(cache) for name, cache in self.buffers.items()},
                "verified_map_bytes": sum(m.byte_count for m in self.maps.values()),
                "maximum_retained_candidates": len(MAP_ORDER) * self.policy.maximum_frames,
                "event_count": len(self.events)}
