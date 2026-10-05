"""New V15 gate: inherited V12 reports never authorize this candidate."""
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def verify_candidate_preflight(profile=None):
    report=json.loads((HERE/'LIO_JACOBIAN_PREFLIGHT.json').read_text())
    if report.get('status')!='PASS_LIMITED_KERNEL_ONLY' or report.get('allowed') is not True:
        raise ValueError('V15 candidate is unverified: independent build and numeric/performance evidence required')
    if report.get('candidate_root')!=str(HERE.resolve()):raise ValueError('V15 candidate identity differs')
    independent=json.loads(Path(report['independent_receipt']).read_text())
    if independent.get('status')!='PASS_LIMITED_KERNEL_ONLY':raise ValueError('Independent limited kernel gate did not pass')
    if report.get('actual_motion_verified') is not False:
        raise ValueError('Preflight must not claim actual V15 motion before its independent run')
    for path,expected in report.get('frozen_references',{}).items():
        if sha(path)!=expected: raise ValueError('V15 preflight source/binary changed '+path)
    required=[HERE/'LIO_JACOBIAN_CONTRACT.json',HERE/'slam_ws/src/fast_livo2_core/src/voxel_map.cpp',
              HERE/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',
              HERE/'slam_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
    if any(str(p.resolve()) not in report.get('frozen_references',{}) for p in required):
        raise ValueError('Incomplete V15 source/binary gate')
    if profile is not None:
        cfg=profile.get('lio_jacobian_parallelism',{})
        if type(cfg.get('threads')) is not int or cfg['threads'] not in (1,4) or cfg.get('parallel_min_rows')!=256:
            raise ValueError('Freeze V15 Jacobian threads 1/4 and min_rows256 explicitly')
        if cfg.get('row_operation_order')!='original_V12' or cfg.get('floating_reduction_changed') is not False:
            raise ValueError('V15 row/matrix order changed')
    return report
