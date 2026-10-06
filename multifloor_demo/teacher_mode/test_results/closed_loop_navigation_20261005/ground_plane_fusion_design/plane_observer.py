"""Pure NumPy point-cloud/map-prior vertical observation; no simulator input.

This file is an offline design, not a runtime navigation adapter. Coordinates
are original registered camera_init coordinates. A positive delta_z means the
observed support plane is below its frozen map-prior height.
"""
import math
import numpy as np


def fit_candidate_planes(points, body_origin, radius=2.5, tolerance=.015,
                         maximum_tilt_deg=15., iterations=300, seed=42):
    points = np.asarray(points, dtype=np.float64)
    origin = np.asarray(body_origin, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or origin.shape != (3,):
        raise ValueError('expected original XYZ and original body-origin XYZ')
    # Only measured geometry selects the ROI. No simulated pose, height,
    # surface name, world collision, or synthetic point is accepted.
    keep = np.isfinite(points).all(axis=1)
    keep &= np.linalg.norm(points[:, :2] - origin[:2], axis=1) <= radius
    keep &= np.abs(points[:, 2] - origin[2]) <= 3.
    remaining = points[keep]
    rng = np.random.default_rng(seed)
    candidates = []
    for candidate_index in range(8):
        if len(remaining) < 80:
            break
        sampled = remaining if len(remaining) <= 2500 else remaining[rng.choice(len(remaining), 2500, replace=False)]
        best = None
        for _ in range(iterations):
            p = sampled[rng.choice(len(sampled), 3, replace=False)]
            n = np.cross(p[1]-p[0], p[2]-p[0])
            norm = np.linalg.norm(n)
            if norm < 1e-8:
                continue
            n /= norm
            if n[2] < 0:
                n = -n
            if n[2] < math.cos(math.radians(maximum_tilt_deg)):
                continue
            d = -float(n @ p[0])
            residual = np.abs(sampled @ n + d)
            mask = residual <= tolerance
            score = int(mask.sum())
            if best is None or score > best[0]:
                best = score, n, d
        if best is None or best[0] < 60:
            break
        _, n, d = best
        for _ in range(3):
            mask = np.abs(remaining @ n + d) <= tolerance
            inliers = remaining[mask]
            if len(inliers) < 80:
                break
            c = inliers.mean(axis=0)
            _, _, vt = np.linalg.svd(inliers-c, full_matrices=False)
            n = vt[-1]
            if n[2] < 0:
                n = -n
            d = -float(n @ c)
        mask = np.abs(remaining @ n + d) <= tolerance
        inliers = remaining[mask]
        if len(inliers) < 80:
            break
        relative_xy = inliers[:, :2]-origin[:2]
        sectors = np.floor((np.arctan2(relative_xy[:, 1], relative_xy[:, 0])+math.pi)/(2*math.pi)*8).astype(int)%8
        sector_count = int(np.sum(np.bincount(sectors, minlength=8) >= 8))
        xy_cov_eigen = np.linalg.eigvalsh(np.cov(relative_xy.T))
        height = float(-(n[:2] @ origin[:2]+d)/n[2])
        clearance = float(origin[2]-height)
        residual = inliers @ n+d
        rms = float(np.sqrt(np.mean(residual**2)))
        tilt = float(math.acos(np.clip(n[2], -1., 1.)))
        supports = bool(.2 <= clearance <= .45 and len(inliers) >= 80
                        and rms <= .015 and sector_count >= 6
                        and xy_cov_eigen[0] >= .04
                        and tilt <= math.radians(maximum_tilt_deg))
        candidates.append(dict(index=candidate_index, normal=n.tolist(), offset=d,
                               height_at_body_xy=height, body_origin_clearance=clearance,
                               inliers=len(inliers), rms_residual_m=rms,
                               tilt_rad=tilt, covered_angular_sectors=sector_count,
                               xy_covariance_eigenvalues=xy_cov_eigen.tolist(),
                               support_eligible=supports))
        remaining = remaining[~mask]
    return candidates


def select_support_plane(candidates):
    """Ambiguity rejects; never selects a plane using desired height bias."""
    eligible = [p for p in candidates if p['support_eligible']]
    if len(eligible) != 1:
        return dict(status='unverified', reason='no unique measured support plane',
                    eligible_count=len(eligible), selected=None)
    return dict(status='observed', reason='unique local support geometry',
                eligible_count=1, selected=eligible[0])


def vertical_observation(selected, initial_plane_height, support_elevation):
    """Map prior only supplies elevation change; it does not pick the plane."""
    if selected['status'] != 'observed':
        return dict(status='unverified', delta_z=None)
    if not np.isfinite([initial_plane_height, support_elevation]).all():
        raise ValueError('nonfinite frozen map-prior input')
    expected = float(initial_plane_height+support_elevation)
    measured = float(selected['selected']['height_at_body_xy'])
    return dict(status='observed', expected_plane_height=expected,
                measured_plane_height=measured, delta_z=expected-measured,
                source='original registered cloud + frozen initial plane + static map elevation',
                simulator_feedback_used=False)
