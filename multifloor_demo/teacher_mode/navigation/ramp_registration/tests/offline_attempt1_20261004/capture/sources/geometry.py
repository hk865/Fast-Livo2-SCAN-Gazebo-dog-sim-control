"""Conservative measured support-plane candidates, never a scene/model filler.

The output is an offline registration candidate, not a traversability certificate
or navigation success. Full candidates need both independently observed support
edges plus actual adjoining platform observations. Cloud hull edges alone fail.
"""
from __future__ import annotations
import numpy as np

DEFAULTS = dict(plane_tolerance_m=.025, min_plane_points=80, ransac_trials=180,
                max_planes=10, max_fit_points=30000, horizontal_max_deg=1.5,
                ramp_min_deg=3., ramp_max_deg=20., bin_m=.25, min_bin_points=6,
                minimum_supported_width_m=1., side_drop_min_m=.06,
                side_evidence_distance_m=.20, seam_tolerance_m=.25,
                minimum_ramp_length_m=1., maximum_edge_spread_m=.12, support_cell_width_m=.10,
                minimum_interior_coverage_fraction=.80, side_face_min_vertical_span_m=.06)


def unit(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all() or np.linalg.norm(value) < 1e-9:
        raise ValueError('invalid measured up axis')
    return value / np.linalg.norm(value)


def extract_planes(xyz, up, cfg):
    """Deterministic, point-order invariant under rigid coordinate transforms."""
    rng = np.random.default_rng(42); left = np.arange(len(xyz)); planes = []; horizontal_phase=True
    while len(planes)<cfg['max_planes']:
        if len(left) < cfg['min_plane_points']: break
        sample = left if len(left) <= cfg['max_fit_points'] else left[np.linspace(0, len(left)-1, cfg['max_fit_points'], dtype=int)]
        points = xyz[sample]; best = None; best_count = 0
        for _trial in range(cfg['ransac_trials']):
            a, b, c = points[rng.choice(len(points), 3, replace=False)]
            n = np.cross(b-a, c-a); length = np.linalg.norm(n)
            if length < 1e-8: continue
            n /= length
            if abs(n @ up) < np.cos(np.deg2rad(cfg['ramp_max_deg'])): continue
            if horizontal_phase and abs(n @ up)<np.cos(np.deg2rad(cfg['horizontal_max_deg'])):continue
            good = np.abs((points-a) @ n) <= cfg['plane_tolerance_m']; count = int(good.sum())
            if count > best_count: best_count, best = count, (a, n)
        if best is None or best_count < cfg['min_plane_points']:
            if horizontal_phase:horizontal_phase=False;continue
            break
        mask = np.abs((xyz[left]-best[0]) @ best[1]) <= cfg['plane_tolerance_m']
        indexes = left[mask]; p = xyz[indexes]; center = p.mean(axis=0)
        normal=best[1]
        for _refine in range(3):
            core=p[np.abs((p-center)@normal)<=cfg['plane_tolerance_m']*.5]
            if len(core)<cfg['min_plane_points']:core=p
            center=core.mean(axis=0);_,_,vh=np.linalg.svd(core-center,full_matrices=False);normal=vh[-1]
        if normal @ up < 0: normal = -normal
        # A thin height slice of a ramp can resemble a horizontal RANSAC
        # proposal. Its fitted normal exposes it; do not remove that slice as
        # a floor before fitting the real ramp.
        if horizontal_phase and normal@up<np.cos(np.deg2rad(cfg['horizontal_max_deg'])):
            horizontal_phase=False;continue
        # Refine the membership after the least-squares fit, retaining every layer.
        mask = np.abs((xyz[left]-center) @ normal) <= cfg['plane_tolerance_m']
        indexes = left[mask]; p = xyz[indexes]
        if len(p) < cfg['min_plane_points']: break
        tilt = float(np.arccos(np.clip(normal @ up, -1, 1)))
        planes.append(dict(plane_id=len(planes), center=center, normal=normal, indices=indexes,
                           tilt_rad=tilt, rms_m=float(np.sqrt(np.mean(((p-center) @ normal)**2)))))
        left = left[~mask]
    return planes


def _public_plane(plane, cfg):
    tilt = np.rad2deg(plane['tilt_rad'])
    kind = ('horizontal_platform_candidate' if tilt <= cfg['horizontal_max_deg'] else
            'ramp_candidate' if cfg['ramp_min_deg'] <= tilt <= cfg['ramp_max_deg'] else 'other_support_candidate')
    return dict(plane_id=plane['plane_id'], kind=kind, center=plane['center'].tolist(),
                normal=plane['normal'].tolist(), tilt_rad=plane['tilt_rad'],
                points=len(plane['indices']), plane_residual_rms_m=plane['rms_m'])


def register(xyz, up, body_anchor, *, cfg=None, source_kind='full_registered_cloud', complete_archive=True):
    cfg = dict(DEFAULTS, **(cfg or {})); up = unit(up); xyz = np.asarray(xyz, dtype=np.float64)
    anchor = np.asarray(body_anchor, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all() or anchor.shape != (3,) or not np.isfinite(anchor).all():
        raise ValueError('finite N×3 measured cloud and body anchor required')
    base = {'schema': 'actual_ramp_registration/v1', 'frame_id': 'camera_init',
            'source_kind': source_kind, 'ground_truth_used': False, 'up_camera_init': up.tolist(),
            'body_anchor': anchor.tolist(), 'configuration': cfg, 'input_points': len(xyz),
            'status': 'unverified', 'full_route_eligible': False, 'full_route': None,
            'support_planes': [], 'ramp_candidates': [], 'rejection_reasons': []}
    if len(xyz) < cfg['min_plane_points']:
        base['rejection_reasons'] = ['insufficient_actual_points']; return base
    planes = extract_planes(xyz, up, cfg); base['support_planes'] = [_public_plane(p, cfg) for p in planes]
    horizontals = [p for p in planes if np.rad2deg(p['tilt_rad']) <= cfg['horizontal_max_deg']]
    current = []
    for p in horizontals:
        pts = xyz[p['indices']]; delta = pts-anchor
        horizontal_dist = np.linalg.norm(delta-(delta @ up)[:, None]*up, axis=1)
        gap = float((anchor-p['center']) @ up)
        if .15 <= gap <= .65 and np.any(horizontal_dist < .45): current.append((gap, p))
    current.sort(key=lambda x: x[0]); current_id = current[0][1]['plane_id'] if current else None
    base['current_support_platform_id'] = current_id
    if len(current) > 1 and abs(current[0][0]-current[1][0]) < .06:
        current_id = None; base['current_support_platform_id'] = None
        base['rejection_reasons'].append('ambiguous_current_support_layer')
    if current_id is None: base['rejection_reasons'].append('current_support_layer_not_observed')
    for p in planes:
        tilt = np.rad2deg(p['tilt_rad'])
        if not cfg['ramp_min_deg'] <= tilt <= cfg['ramp_max_deg']: continue
        heading = up-(up @ p['normal'])*p['normal']; heading -= (heading @ up)*up; heading = unit(heading)
        lateral = unit(np.cross(up, heading)); center = p['center']; delta = xyz-center
        u, v, h = delta @ heading, delta @ lateral, delta @ up
        # Reconsider original measurements near a fitted ramp, including seam
        # samples that a horizontal component also legitimately owns.
        pts=xyz[np.abs((xyz-center)@p['normal'])<=cfg['plane_tolerance_m']]
        uv = (pts-center) @ np.column_stack([heading, lateral])
        low, high = np.quantile(uv[:, 0], [.005, .995]); length = float(high-low)
        slope = float(-(p['normal'] @ heading)/(p['normal'] @ up))
        # Clip seam-contaminated plane membership only when both actual
        # adjoining platform levels and center-strip measurements support it.
        tentative=[];middle0=float(np.median(uv[:,1]))
        for plat in horizontals:
            seam=float((plat['center']-center)@up)/slope
            if not low-cfg['seam_tolerance_m']<=seam<=high+cfg['seam_tolerance_m']:continue
            kind='entry_low'if abs(seam-low)<=cfg['seam_tolerance_m']else'exit_high'if abs(seam-high)<=cfg['seam_tolerance_m']else None
            if kind is None:continue
            pu=(xyz[plat['indices']]-center)@heading;pv=(xyz[plat['indices']]-center)@lateral
            band=(pu>=seam-.45)&(pu<=seam-.05)if kind=='entry_low'else(pu<=seam+.45)&(pu>=seam+.05)
            if (band&(np.abs(pv-middle0)<=.4)).sum()>=cfg['min_bin_points']:tentative.append((kind,seam))
        entries=[s for kind,s in tentative if kind=='entry_low'];exits=[s for kind,s in tentative if kind=='exit_high']
        if len(entries)==len(exits)==1:low,high=entries[0],exits[0];length=float(high-low)
        candidate = {'plane_id': p['plane_id'], 'heading_camera_init': heading.tolist(),
                     'lateral_camera_init': lateral.tolist(), 'observed_u_interval_m': [float(low), float(high)],
                     'observed_length_m': length, 'gradient': slope, 'bins': [], 'connections': [],
                     'status': 'partial', 'rejection_reasons': [], 'full_route': None}
        base['ramp_candidates'].append(candidate)
        if length < cfg['minimum_ramp_length_m']:
            candidate['rejection_reasons'].append('observed_ramp_too_short'); continue
        nbin = max(1, int(np.ceil(length/cfg['bin_m']))); boundaries = np.linspace(low, high, nbin+1)
        for j, (a, b) in enumerate(zip(boundaries[:-1], boundaries[1:])):
            surf = (uv[:, 0] >= a) & (uv[:, 0] <= b)
            vs = uv[surf, 1]; row = dict(bin=j, u_interval_m=[float(a), float(b)], support_points=int(len(vs)),
                                         bilateral_observed=False, left_edge_m=None, right_edge_m=None)
            if len(vs) >= cfg['min_bin_points']:
                left, right = np.quantile(vs, [.02, .98]); width = float(right-left)
                common = (u >= a-cfg['bin_m']*.1) & (u <= b+cfg['bin_m']*.1)
                below = h < slope*u-cfg['side_drop_min_m']
                left_mask = common & below & (v < left-.015) & (v >= left-cfg['side_evidence_distance_m'])
                right_mask = common & below & (v > right+.015) & (v <= right+cfg['side_evidence_distance_m'])
                # A lower layer alone can be visible through a sampling hole.
                # Require measured near-vertical edge-face returns connecting
                # the surface and lower side, rather than certifying a hull.
                depth = slope*u-h
                near_surface_side = common & (depth >= .01) & (depth <= .22)
                left_face = depth[near_surface_side & (np.abs(v-left) <= .05)]
                right_face = depth[near_surface_side & (np.abs(v-right) <= .05)]
                face_spans = [float(np.ptp(a)) if len(a)>=3 else 0. for a in (left_face,right_face)]
                cells = max(1, int(np.ceil(width/cfg['support_cell_width_m'])))
                occupied_cells = len(np.unique(np.clip(((vs-left)/max(width,1e-9)*cells).astype(int),0,cells-1)))
                interior = occupied_cells/cells
                row.update(left_edge_m=float(left), right_edge_m=float(right), observed_width_m=width,
                           left_external_drop_points=int(left_mask.sum()), right_external_drop_points=int(right_mask.sum()),
                           left_actual_side_face_vertical_span_m=face_spans[0],right_actual_side_face_vertical_span_m=face_spans[1],
                           interior_observed_support_fraction=interior,
                           bilateral_observed=bool(width >= cfg['minimum_supported_width_m']
                               and left_mask.sum() >= 2 and right_mask.sum() >= 2
                               and min(face_spans)>=cfg['side_face_min_vertical_span_m']
                               and interior>=cfg['minimum_interior_coverage_fraction']))
            candidate['bins'].append(row)
        good = [r for r in candidate['bins'] if r['bilateral_observed']]
        candidate['bilateral_bins'] = len(good); candidate['total_bins'] = len(candidate['bins'])
        candidate['bilateral_coverage_fraction'] = len(good)/len(candidate['bins'])
        if not good:
            candidate['rejection_reasons'].append('no_actual_double_sided_support_boundary')
            continue
        left = float(np.median([r['left_edge_m'] for r in good])); right = float(np.median([r['right_edge_m'] for r in good]))
        middle = (left+right)*.5
        spread = max(float(np.ptp([r['left_edge_m'] for r in good])), float(np.ptp([r['right_edge_m'] for r in good])))
        candidate['edge_spread_m'] = spread; candidate['centerline_lateral_m'] = middle
        if spread > cfg['maximum_edge_spread_m']: candidate['rejection_reasons'].append('support_edges_not_consistent_straight_corridor')
        for plat in horizontals:
            level = float((plat['center']-center) @ up); seam = level/slope
            if not low-cfg['seam_tolerance_m'] <= seam <= high+cfg['seam_tolerance_m']: continue
            pu = (xyz[plat['indices']]-center) @ heading; pv = (xyz[plat['indices']]-center) @ lateral
            end = 'entry_low' if abs(seam-low) <= cfg['seam_tolerance_m'] else 'exit_high' if abs(seam-high) <= cfg['seam_tolerance_m'] else None
            if end is None: continue
            outside = ((pu >= seam-.45) & (pu <= seam-.05) if end=='entry_low' else (pu <= seam+.45) & (pu >= seam+.05))
            measured_platform = outside & (np.abs(pv-middle) <= .40)
            if measured_platform.sum() >= cfg['min_bin_points']:
                candidate['connections'].append(dict(platform_id=plat['plane_id'], end=end,
                    u_m=float(seam), platform_observed_points=int(measured_platform.sum())))
        entry = [r for r in candidate['connections'] if r['end']=='entry_low']
        exit_ = [r for r in candidate['connections'] if r['end']=='exit_high']
        if len(entry)!=1 or len(exit_)!=1: candidate['rejection_reasons'].append('both_adjoining_platform_seams_not_observed_or_ambiguous')
        if len(good) != len(candidate['bins']): candidate['rejection_reasons'].append('partial_double_boundary_coverage')
        if current_id not in {r['platform_id'] for r in candidate['connections']}:
            candidate['rejection_reasons'].append('ramp_not_connected_to_current_observed_support_layer')
        if source_kind != 'full_registered_cloud': candidate['rejection_reasons'].append('exploratory_color_cloud_cannot_certify_full_coverage')
        if not complete_archive: candidate['rejection_reasons'].append('input_archive_incomplete')
        if not candidate['rejection_reasons']:
            seam_points = [center + heading*r[0]['u_m'] + lateral*middle + up*slope*r[0]['u_m'] for r in (entry, exit_)]
            if current_id == exit_[0]['platform_id']: seam_points.reverse()
            clearance = current[0][0]
            candidate['status'] = 'full_candidate'; candidate['full_route'] = {
                'surface_centerline_camera_init': [q.tolist() for q in seam_points],
                'body_centerline_camera_init': [(q+up*clearance).tolist() for q in seam_points],
                'measured_initial_body_surface_clearance_m': clearance,
                'platform_ids': [r[0]['platform_id'] for r in (entry, exit_)],
                'scope': 'observed ramp between actual platform seams; not a collision or locomotion acceptance',
                'navigation_ground_truth_used': False}
    full = [r for r in base['ramp_candidates'] if r['status']=='full_candidate']
    if len(full)==1 and not base['rejection_reasons']:
        base['status']='full_candidate'; base['full_route_eligible']=True; base['full_route']=full[0]['full_route']
    elif base['ramp_candidates']:
        base['status']='partial'
        if len(full)>1: base['rejection_reasons'].append('multiple_connected_ramps_require_measured_mission_selection')
    else:
        base['rejection_reasons'].append('no_supported_ramp_plane_observed')
    return base
