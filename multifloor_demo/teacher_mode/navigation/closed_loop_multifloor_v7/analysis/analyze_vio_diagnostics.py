#!/usr/bin/env python3
"""Read-only V7 diagnostic analysis. Does not start ROS/Gazebo or alter inputs.

Usage: python3 analyze_vio_diagnostics.py RUN --output NEW_DIRECTORY
       python3 analyze_vio_diagnostics.py --self-test
Outputs are new derived artifacts. Original binary, runtime, gates and receipts
are never modified. Native/Gazebo state is not read by this analyzer.
"""
import os
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'
import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation

MAGIC = b'FLIVODIAG0001LE\0'
HEADER = struct.Struct('<7Q')
BASE = Path(__file__).resolve().parents[1]


def clean(value):
    if isinstance(value, dict): return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, np.generic): return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean(value), indent=2, ensure_ascii=False) + '\n')


def i64(value): return value if value < 2**63 else value - 2**64


def records(path, meta, allow_partial=False):
    # Large point/query/cloud records are intentionally not read. The writer's
    # append-only index lets this analyzer seek only the necessary small matrix,
    # state, IMU and VIO pixel records; the full binary hash belongs to archive
    # provenance, not this selected-record reconstruction.
    selected={1,10,11,12,13,20,100,200,201,202,203}
    digest=hashlib.sha256();index=path.parent/'index.csv'
    with path.open('rb') as source:
        magic=source.read(16);digest.update(magic)
        if magic!=MAGIC:raise ValueError(f'Unsupported diagnostic magic: {magic!r}')
        meta.update(bytes_read=16,record_count=0,selected_record_count=0,partial_tail=False,binary_bytes=path.stat().st_size,index_used=index.exists(),
                    hash_scope='16-byte magic plus selected complete record headers and payloads in original order; not a full binary hash')
        def indexed_headers():
            with index.open(newline='') as stream:
                for row in csv.DictReader(stream):
                    try: values=[int(row[k]) for k in ('kind','sequence','stamp_ns','stage','iteration','level','n_values')];offset=int(row['offset'])
                    except (TypeError,ValueError,KeyError):
                        if not allow_partial:raise ValueError('Malformed diagnostic index row')
                        meta['partial_tail']=True;break
                    yield offset,values
        def streamed_headers():
            source.seek(16)
            while True:
                offset=source.tell();raw=source.read(HEADER.size)
                if not raw:break
                if len(raw)!=HEADER.size:
                    if not allow_partial:raise ValueError(f'Truncated header at {offset}')
                    meta['partial_tail']=True;break
                values=list(HEADER.unpack(raw));source.seek(values[6]*8,1)
                yield offset,values
        headers=indexed_headers() if index.exists() else streamed_headers()
        for offset,values in headers:
            meta['record_count']+=1
            count=values[6]
            if count>8_000_000:raise ValueError(f'Unsafe/corrupt count at {offset}: {count}')
            if offset+HEADER.size+count*8>meta['binary_bytes']:
                if not allow_partial:raise ValueError(f'Truncated payload at {offset}')
                meta['partial_tail']=True;break
            if values[0] not in selected:continue
            source.seek(offset);raw=source.read(HEADER.size)
            if list(HEADER.unpack(raw))!=values:raise ValueError(f'Index/header mismatch at {offset}')
            payload=source.read(count*8)
            if len(payload)!=count*8:
                if not allow_partial:raise ValueError(f'Truncated selected payload at {offset}')
                meta['partial_tail']=True;break
            digest.update(raw);digest.update(payload);meta['bytes_read']+=len(raw)+len(payload)
            header=values.copy();header[4],header[5]=i64(header[4]),i64(header[5]);meta['selected_record_count']+=1
            yield header,np.frombuffer(payload,dtype='<f8'),offset
            # For the non-index fixture reader restore its original next header.
            if not index.exists():source.seek(offset+HEADER.size+count*8)
        meta['selected_record_bytes_sha256']=digest.hexdigest()


class Checks:
    def __init__(self): self.items = {}
    def compare(self, name, actual, expected, context, rtol=2e-7, atol=2e-9):
        actual, expected = np.asarray(actual), np.asarray(expected)
        valid = actual.shape == expected.shape and np.isfinite(actual).all() and np.isfinite(expected).all()
        error = float(np.max(np.abs(actual - expected))) if valid and actual.size else np.nan
        okay = valid and np.allclose(actual, expected, rtol=rtol, atol=atol)
        self.add(name, okay, context, error)
        return error
    def add(self, name, okay, context, error=np.nan):
        row = self.items.setdefault(name, dict(passed=0, failed=0, maximum_absolute_error=0.0, failure_examples=[]))
        row['passed' if okay else 'failed'] += 1
        if np.isfinite(error): row['maximum_absolute_error'] = max(row['maximum_absolute_error'], error)
        if not okay and len(row['failure_examples']) < 8: row['failure_examples'].append(dict(context=context, absolute_error=error))
    def okay(self): return bool(self.items) and not any(v['failed'] for v in self.items.values())


def state_minus(a, b):
    """Actual common_lib.h and so3_math.h right tangent, including small Log."""
    R=b[:9].reshape(3,3).T@a[:9].reshape(3,3)
    theta=0.0 if R.trace()>3.0-1e-6 else np.arccos(.5*(R.trace()-1.0))
    K=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    tangent=.5*K if abs(theta)<.001 else .5*theta/np.sin(theta)*K
    return np.r_[tangent,a[9:]-b[9:]]


def state_plus(a, delta):
    # The actual Exp(v1,v2,v3) returns identity at norm <= 1e-5 rad.
    # Do not replace this source behavior with a mathematical ideal exponential.
    norm=np.linalg.norm(delta[:3]);rotation=np.eye(3)
    if norm>1e-5:
        x,y,z=delta[:3]/norm;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        rotation=rotation+np.sin(norm)*K+(1-np.cos(norm))*(K@K)
    return np.r_[(a[:9].reshape(3,3)@rotation).ravel(),a[9:]+delta[3:]]


def information(M):
    """Offline eig/rank diagnostics, never a solver/acceptance rule."""
    M6 = M[:6,:6]
    if not np.isfinite(M6).all(): return dict(Mzz=np.nan, rank=np.nan, eig_min=np.nan, eig_max=np.nan, schur_z=np.nan)
    symmetric = (M6 + M6.T) * .5
    eig = np.linalg.eigvalsh(symmetric)
    threshold = max(abs(eig[-1]) * 1e-9, 1e-12)
    other = [0,1,2,3,4]
    schur = symmetric[5,5] - symmetric[5,other] @ np.linalg.pinv(symmetric[np.ix_(other,other)], rcond=1e-9) @ symmetric[other,5]
    return dict(Mzz=float(M6[5,5]), rank=int((eig>threshold).sum()), eig_min=float(eig[0]), eig_max=float(eig[-1]), schur_z=float(schur), asymmetry_max=float(np.max(abs(M6-M6.T))))


def parse_iteration(d):
    if len(d) == 3: return dict(skipped=True, branch=int(d[0]), hdim=int(d[1]), skip=int(d[2]))
    hd = int(d[1]); expected = 12+25+25+361+hd*hd+hd+19+25+361
    if hd not in (6,7) or len(d) != expected: raise ValueError(f'Bad kind201 size/hd: {len(d)}/{hd}, expected {expected}')
    at=12
    def take(n):
        nonlocal at
        value=d[at:at+n]; at+=n; return value
    return dict(skipped=False, metadata=d[:12], prop=take(25), before=take(25), P=take(361).reshape(19,19),
                M=take(hd*hd).reshape(hd,hd), ht=take(hd), solution=take(19), after=take(25), Pafter=take(361).reshape(19,19))


def recover_solution(item):
    """Rebuild original K1/G form from saved M=HtH/R, ht=HTz/R."""
    hd=len(item['ht']); R=item['metadata'][5]
    H=np.zeros((19,19)); H[:hd,:hd]=item['M']*R
    K1=np.linalg.inv(H + np.linalg.inv(item['P']/R))
    G=K1[:,:hd] @ H[:hd,:hd]
    vec=state_minus(item['prop'],item['before'])
    solution=-K1[:,:hd] @ (item['ht']*R) + vec - G @ vec[:hd]
    return solution, G


def stats(values):
    a=np.asarray(values,dtype=float); a=a[np.isfinite(a)]
    return dict(count=len(a), minimum=float(a.min()) if len(a) else None, median=float(np.median(a)) if len(a) else None,
                mean=float(a.mean()) if len(a) else None, maximum=float(a.max()) if len(a) else None)


def analyze(binary, output, detail_begin=115.0, detail_end=165.0, windows=None, allow_partial=False, writer_stats=None, max_iterations=5):
    output.mkdir(parents=True, exist_ok=False)
    checks=Checks(); counts=Counter(); binary_meta={}; phase_groups=defaultdict(dict)
    vio_rows=[]; lio_rows=[]; prop_rows=[]; retrieval_rows=[]; track_rows=[]; management=[]; imu_rows=[]
    matrix_expectations={}; level_rollback={}; last_gain={}; frame_first_state={}; frame_last_state={}; expected_pixels=set(); observed_pixels=set()
    track_counts=Counter(); tracks_expected={}; discarded_detail=0
    lio_schema=json.loads((BASE/'lio_diagnostic_schema.json').read_text())
    lio_fields={f['name']:f for f in lio_schema['kinds']['100']['fields']}
    def lio_field(d,name):
        f=lio_fields[name]; v=d[f['offset']:f['offset']+f['count']]
        return v.reshape(f['shape']) if f['shape'] else v[0]
    for h,d,offset in records(binary,binary_meta,allow_partial):
        kind,seq,stamp,stage,it,level,_=h; counts[kind]+=1
        context=dict(sequence=seq,stamp_ns=stamp,stage=stage,iteration=it,level=level,offset=offset)
        key=(seq,it,level)
        if kind==201:
            item=parse_iteration(d)
            if item['skipped']:
                vio_rows.append(dict(**context,skip_code=item['skip'],accepted=None,total_points=0,n_meas=0)); continue
            m=item['metadata']; hd=len(item['ht']); accepted=bool(m[9]); rollback_key=(seq,level)
            if seq not in frame_first_state:frame_first_state[seq]=item['before'].copy()
            frame_last_state[seq]=item['after'].copy()
            if rollback_key not in level_rollback: level_rollback[rollback_key]=item['before'].copy()
            gain_error=np.nan; solution_error=np.nan; recovered_error=np.nan
            checks.compare('iteration_covariance_unchanged',item['Pafter'],item['P'],context,rtol=0,atol=0)
            checks.add('post_update_error_explicitly_not_computed', np.isnan(m[8]),context)
            checks.add('actual_acceptance_branch_matches_error_test', accepted == bool(m[7]<=m[6]),context)
            if accepted:
                checks.compare('accepted_solution_matches_state_delta',item['after'],state_plus(item['before'],item['solution']),context)
                level_rollback[rollback_key]=item['before'].copy()
                try:
                    reconstructed,G=recover_solution(item)
                    recovered_error=checks.compare('solution_reconstructed_from_information_prior',item['solution'],reconstructed,context,rtol=2e-6,atol=2e-8)
                    last_gain[seq]=(hd,G)
                except (ValueError,np.linalg.LinAlgError) as error:
                    checks.add('solution_reconstructed_from_information_prior',False,dict(context,error=str(error)))
            else:
                checks.compare('rollback_restores_original_old_state',item['after'],level_rollback[rollback_key],context)
                checks.add('rollback_solution_absent',np.isnan(item['solution']).all(),context)
            info=information(item['M'])
            vio_rows.append(dict(**context,skip_code=int(m[2]),accepted=int(accepted),total_points=int(m[3]),n_meas=int(m[4]),R=m[5],
                error_previous=m[6],error_candidate=m[7],end=int(m[10]),solution_reconstruction_error=recovered_error,
                delta_z=item['after'][11]-item['before'][11],delta_vz=item['after'][15]-item['before'][15],
                z_before=item['before'][11],z_after=item['after'][11],vz_before=item['before'][15],vz_after=item['after'][15],
                P_pz_vz=item['P'][5,9],**info))
            # Only compact matrices needed to pair detail H/z; no full-P retention.
            if detail_begin<=stamp*1e-9<=detail_end and (it==0 or bool(m[10]) or it==max_iterations-1):
                expected_pixels.add(key); matrix_expectations[key]=(item['M'].copy(),item['ht'].copy(),int(m[4]),hd,m[5])
        elif kind==203:
            if len(d)<5: raise ValueError('Short kind203')
            rows,cols,n_meas=int(d[1]),int(d[2]),int(d[3]); R=d[4]
            if len(d)!=5+rows*cols+rows: raise ValueError('Wrong kind203 array size')
            observed_pixels.add(key)
            if key not in matrix_expectations:
                checks.add('pixel_arrays_have_same_iteration_aggregate',False,context); continue
            M,ht,expected_meas,hd,expected_R=matrix_expectations.pop(key)
            H=d[5:5+rows*cols].reshape(rows,cols); z=d[5+rows*cols:]
            checks.add('pixel_metadata_matches_aggregate',cols==hd and n_meas==expected_meas and R==expected_R,context)
            checks.compare('HtH_reconstructed_from_actual_pixels',M,H.T@H/R,context,rtol=3e-7,atol=3e-8)
            checks.compare('HTz_reconstructed_from_actual_pixels',ht,H.T@z/R,context,rtol=3e-7,atol=3e-8)
        elif kind==200:
            phase=int(d[0])
            if phase==1:
                if len(d)!=24:raise ValueError('Bad retrieval summary')
                row=dict(**context,skip_code=int(d[1]),frame_id=int(d[2]),pg_count=int(d[3]),map_voxels=int(d[4]),map_points=int(d[5]),
                    query_voxels=int(d[6]),candidate_visits=int(d[7]),reject_null=int(d[8]),reject_empty_obs=int(d[9]),reject_behind=int(d[10]),
                    fov_passes=int(d[11]),selected_grid=int(d[12]),reject_depth=int(d[13]),reject_normal=int(d[14]),reference_passes=int(d[15]),
                    reject_SSE=int(d[16]),accepted_tracks=int(d[17]),ncc_evaluations=d[18],ncc_enabled=int(d[19]),reject_NCC=d[20],normal_enabled=int(d[21]),raycast_enabled=int(d[22]),total_points=int(d[23]))
                row['reject_outside_fov']=row['candidate_visits']-row['reject_null']-row['reject_empty_obs']-row['reject_behind']-row['fov_passes']
                row['reject_reference_selection']=row['selected_grid']-row['reject_depth']-row['reject_normal']-row['reference_passes']
                retrieval_rows.append(row)
                if not row['ncc_enabled']:checks.add('disabled_NCC_is_not_evaluated',np.isnan(d[18]) and np.isnan(d[20]),context)
                if detail_begin<=stamp*1e-9<=detail_end:tracks_expected[seq]=row['accepted_tracks']
            elif phase==3:
                if len(d)!=1139:raise ValueError('Bad frame covariance summary')
                before=d[6:367].reshape(19,19);state_before=d[367:392];after=d[392:753].reshape(19,19);state_after=d[753:778];G=d[778:].reshape(19,19)
                if int(d[1])==3:
                    checks.compare('zero_track_frame_state_unchanged',state_before,state_after,context,rtol=0,atol=0)
                else:
                    checks.add('frame_state_has_actual_iteration_pair',seq in frame_first_state and seq in frame_last_state,context)
                    if seq in frame_first_state:checks.compare('frame_initial_state_matches_first_iteration',state_before,frame_first_state.pop(seq),context,rtol=0,atol=0)
                    if seq in frame_last_state:checks.compare('frame_final_state_matches_last_iteration',state_after,frame_last_state.pop(seq),context,rtol=0,atol=0)
                if int(d[1])==3:
                    checks.compare('zero_track_frame_covariance_unchanged',before,after,context,rtol=0,atol=0)
                else:
                    checks.compare('actual_frame_covariance_P_minus_GP',after,before-G@before,context,rtol=3e-7,atol=3e-10)
                    if seq in last_gain:
                        hd,gain=last_gain.pop(seq)
                        expected_G=np.zeros((19,19));expected_G[:,:hd]=gain
                        checks.compare('frame_gain_matches_latest_accepted_iteration',G,expected_G,context,rtol=3e-6,atol=3e-8)
            elif phase in (5,6):management.append(dict(**context,phase=phase,skip_code=int(d[1]),frame_id=int(d[2]),actual_added=int(d[3]),count_after=int(d[4])))
        elif kind==202:
            if len(d)!=36:raise ValueError('Bad track length')
            track_counts[seq]+=1
            checks.add('actual_track_frame_age_matches_source',np.isnan(d[4]) or np.isclose(d[4],stamp*1e-9-d[3],atol=1e-9,rtol=0),context)
            if not int(d[10]):checks.add('disabled_track_NCC_is_NaN',np.isnan(d[9]),context)
            track_rows.append(dict(**context,point_id=int(d[0]),frame_id=int(d[1]),reference_frame_id=int(d[2]),reference_stamp_s=d[3],reference_age_s=d[4],observation_count=int(d[5]),search_level=int(d[6]),reference_level=int(d[7]),SSE=d[8],NCC=d[9],ncc_enabled=int(d[10]),point_x=d[12],point_y=d[13],point_z=d[14],pixel_u=d[15],pixel_v=d[16]))
        elif kind in (10,11,12,13):
            offsets={10:14,11:5,12:2,13:2}; widths={10:400,11:391,12:388,13:388}
            if len(d)!=widths[kind]:raise ValueError(f'Bad caller phase length {kind}: {len(d)}')
            at=offsets[kind];state=d[at:at+25].copy();P=d[at+25:].reshape(19,19).copy()
            phase_groups[seq].update(context=context)
            phase_groups[seq][kind]=dict(state=state,P=P,processed=int(d[0]) if kind in (12,13) else None)
        elif kind==20:
            if len(d)!=58:raise ValueError('Bad propagation pair length')
            dt=d[4];az=d[42];z_before=d[30];z_after=d[48];vz_before=d[27];vz_after=d[45]
            checks.compare('IMU_pair_discrete_position_update',[z_after-z_before],[vz_before*dt+.5*az*dt*dt],context,rtol=2e-6,atol=2e-10)
            checks.compare('IMU_pair_discrete_velocity_update',[vz_after-vz_before],[az*dt],context,rtol=2e-6,atol=2e-10)
            prop_rows.append(dict(**context,head_s=d[0],tail_s=d[1],dt=dt,world_az=az,z_before=z_before,z_after=z_after,vz_before=vz_before,vz_after=vz_after,dz_velocity=vz_before*dt,dz_acceleration=.5*az*dt*dt,dz=z_after-z_before,dvz=vz_after-vz_before))
        elif kind==100:
            if len(d)!=lio_schema['kinds']['100']['payload_width']:raise ValueError('Bad fixed LIO aggregate length')
            before=lio_field(d,'state_iter_before');after=lio_field(d,'state_iter_after');P=lio_field(d,'P_before');M=lio_field(d,'M6');sol=lio_field(d,'solution19')
            lio_rows.append(dict(**context,effective_count=int(lio_field(d,'effective_count')),candidate_count=int(lio_field(d,'downsampled_count')),
                delta_z=after[11]-before[11],delta_vz=after[15]-before[15],solution_pz=sol[5],solution_vz=sol[9],
                HTz_z=lio_field(d,'HTz6')[5],P_pz_vz=P[5,9],stop=int(lio_field(d,'stop')),**information(M)))
        elif kind==1:
            if len(d)!=42:raise ValueError('Bad raw IMU length')
            imu_rows.append(dict(stamp_ns=stamp,receipt=d[0],gyro_norm=float(np.linalg.norm(d[5:8])),accel_norm=float(np.linalg.norm(d[8:11])),ax=d[8],ay=d[9],az=d[10]))
    for key in sorted(expected_pixels-observed_pixels):checks.add('expected_detail_pixels_present',False,dict(sequence=key[0],iteration=key[1],level=key[2]))
    if expected_pixels:checks.add('detail_pixel_arrays_available',bool(observed_pixels),dict(expected=len(expected_pixels),observed=len(observed_pixels)))
    for seq,expected in tracks_expected.items():checks.add('accepted_track_detail_count_matches_retrieval',track_counts[seq]==expected,dict(sequence=seq,expected=expected,actual=track_counts[seq]))
    phase_rows=[]
    for seq,group in sorted(phase_groups.items()):
        c=group['context']; before=group.get(10);prediction=group.get(11);final=group.get(12) or group.get(13)
        if not before or not prediction:continue
        b,p=before['state'],prediction['state'];f=final['state'] if final else np.full(25,np.nan)
        phase_rows.append(dict(**c,complete=int(final is not None),processed=final['processed'] if final else None,
            propagation_dz=p[11]-b[11],propagation_dvz=p[15]-b[15],measurement_dz=f[11]-p[11],measurement_dvz=f[15]-p[15],
            z_before=b[11],z_predicted=p[11],z_final=f[11],vz_before=b[15],vz_predicted=p[15],vz_final=f[15],
            P_pz_vz_before=before['P'][5,9],P_pz_vz_predicted=prediction['P'][5,9],P_pz_vz_final=final['P'][5,9] if final else np.nan))
    def output_csv(name,rows):
        if not rows:return
        fields=list(dict.fromkeys(key for row in rows for key in row))
        with (output/name).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    for name,rows in [('vio_iterations.csv',vio_rows),('lio_information_comparison.csv',lio_rows),('vio_retrieval.csv',retrieval_rows),('vio_tracks.csv',track_rows),('vio_map_management.csv',management),('estimator_stages.csv',phase_rows),('imu_pair_propagation.csv',prop_rows),('raw_imu.csv',imu_rows)]:output_csv(name,rows)
    windows=windows or [(115,125),(125,135),(135,145),(145,155),(155,165),(165,175)]
    window_metrics=[]
    for lo,hi in windows:
        low,high=round(lo*1e9),round(hi*1e9)
        choose=lambda rows:[r for r in rows if low<r['stamp_ns']<=high]
        pr=choose(phase_rows);vr=choose(vio_rows);lr=choose(lio_rows);tr=choose(track_rows);rr=choose(retrieval_rows);ir=choose(imu_rows);ur=choose(prop_rows);mr=choose(management)
        phase_summary={}
        for stage,name in [(2,'LIO'),(1,'VIO')]:
            rows=[r for r in pr if r['stage']==stage and r['complete']]
            phase_summary[name]=dict(complete_stage_count=len(rows),propagation_dz_sum_m=sum(r['propagation_dz'] for r in rows),measurement_dz_sum_m=sum(r['measurement_dz'] for r in rows),propagation_dvz_sum_mps=sum(r['propagation_dvz'] for r in rows),measurement_dvz_sum_mps=sum(r['measurement_dvz'] for r in rows),incomplete_stage_count=sum(r['stage']==stage and not r['complete'] for r in pr))
        worst=sorted([r for r in pr if np.isfinite(r['measurement_dvz'])],key=lambda r:r['measurement_dvz'])[:3]
        window_metrics.append(dict(window_start_exclusive_end_inclusive_s=[lo,hi],phase_sums=phase_summary,largest_downward_velocity_updates=worst,
            actual_tracks=stats([r['total_points'] for r in rr]),reference_age_s=stats([r['reference_age_s'] for r in tr]),observations_per_track=stats([r['observation_count'] for r in tr]),
            added_visual_points=sum(r['actual_added'] for r in mr if r['phase']==5),added_reference_observations=sum(r['actual_added'] for r in mr if r['phase']==6),
            VIO_Mzz=stats([r.get('Mzz') for r in vr if r.get('Mzz') is not None]),VIO_conditional_z_information=stats([r.get('schur_z') for r in vr if r.get('schur_z') is not None]),
            LIO_Mzz=stats([r['Mzz'] for r in lr]),LIO_conditional_z_information=stats([r['schur_z'] for r in lr]),LIO_effective_constraints=stats([r['effective_count'] for r in lr]),
            raw_IMU_gyro_norm=stats([r['gyro_norm'] for r in ir]),raw_IMU_accel_norm=stats([r['accel_norm'] for r in ir]),
            actual_IMU_propagation_dz_from_velocity=sum(r['dz_velocity'] for r in ur),actual_IMU_propagation_dz_from_acceleration=sum(r['dz_acceleration'] for r in ur),
            actual_IMU_propagation_dvz=sum(r['dvz'] for r in ur),VIO_accepted_iterations=sum(bool(r.get('accepted')) for r in vr),VIO_rollback_iterations=sum(r.get('accepted')==0 for r in vr)))
    writer=writer_stats or {}
    complete=not binary_meta.get('partial_tail') and writer.get('final') is True and writer.get('dropped')==0 and writer.get('writer_io_failed') is False
    result=dict(schema='fastlivo_v7_vio_diagnostic_analysis/v1',binary=str(binary),analysis_source=str(Path(__file__).resolve()),analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source=binary_meta,analysis_semantics={'state_operator':'exact frozen so3_math.h Exp identity deadband <=1e-5 rad and Log small-angle branch, not ideal SciPy exponential','kind200_phase3_state_before':'before all VIO levels; P_before is prior covariance which remains unchanged through iterations','kind200_phase3_state_after':'after all levels and final covariance update; validated against final iteration state'},record_counts=dict(counts),writer_stats=writer,diagnostic_capture_complete=complete,algebra_checks_passed=checks.okay(),checks=checks.items,
        navigation_or_motion_acceptance_changed=False,native_or_Gazebo_pose_read=False,detail_window_s=[detail_begin,detail_end],windows=window_metrics,
        limitations=['Algebra checks only validate diagnostic reconstruction and original numerical updates; they do not make navigation or Sim2Sim pass.',
            'VIO information eigenvalues/rank derive from actual saved pixel Jacobians; mixed rotation-radian/translation-meter units use a 1 m reference length and relative rank threshold 1e-9. They are not full fused SLAM observability or proof of correct association.',
            'Conditional z information uses symmetric offline M6 and pseudoinverse of other pose DOFs at rcond1e-9; original matrix bytes are preserved in binary.',
            'Point/ref statistics are actual retrieval/update values; no raw patch imagery or removal-count trace is synthesized.',
            'phase sums use exact absolute source stamp and sequence; no nearest-time or hard-coded debug offset. LIO final published state and post-VIO state remain distinct.',
            'Missing/dropped/incomplete writer capture prevents complete diagnostic claims; --allow-partial explicitly marks partial results.'])
    write_json(output/'summary.json',result)
    return result


def pack(kind,sequence,stamp,stage,it,level,data):
    a=np.asarray(data,dtype='<f8');return HEADER.pack(kind,sequence,stamp,stage,it%(2**64),level%(2**64),len(a))+a.tobytes()


def self_test():
    rng=np.random.default_rng(42);R=1000.;H=rng.normal(size=(16,7));z=rng.normal(size=16);P=np.eye(19)*.005;P[5,9]=P[9,5]=.001
    before=np.r_[np.eye(3).ravel(),[1.,2.,3.],1.,[.1,.2,.3],[0.,0.,0.],[0.,0.,0.],[0.,0.,-9.81]]
    prop=state_plus(before,np.r_[.001,.002,.003,np.zeros(16)])
    item=dict(metadata=np.r_[0,7,0,1,16,R,1e9,.2,np.nan,1,0,16],prop=prop,before=before,P=P,M=H.T@H/R,ht=H.T@z/R)
    solution,gain=recover_solution(item);after=state_plus(before,solution);G=np.zeros((19,19));G[:,:7]=gain
    iteration=lambda metadata,b,a,sol:np.r_[metadata,prop,b,P.ravel(),item['M'].ravel(),item['ht'],sol,a,P.ravel()]
    d1=iteration(item['metadata'],before,after,solution)
    metadata2=np.r_[0,7,0,1,16,R,.2,.3,np.nan,0,1,16]
    d2=iteration(metadata2,after,before,np.full(19,np.nan))
    cov=np.r_[3,0,1,1,R,0,P.ravel(),before,(P-G@P).ravel(),before,G.ravel()]
    retrieval=np.zeros(24);retrieval[:6]=[1,0,1,1,1,1];retrieval[7:13]=[1,0,0,0,1,1];retrieval[15]=1;retrieval[17]=retrieval[23]=1;retrieval[18]=retrieval[20]=np.nan;retrieval[21]=1
    track=np.r_[1,1,1,115.,5.,1,0,0,.2,np.nan,0,1,np.zeros(24)]
    phase_before=np.r_[np.zeros(14),before,P.ravel()];phase_pred=np.r_[np.zeros(5),before,P.ravel()];phase_final=np.r_[1,0,before,P.ravel()]
    raw=bytearray(MAGIC)
    raw+=pack(10,1,120_000_000_000,1,-1,-1,phase_before)+pack(11,1,120_000_000_000,1,-1,-1,phase_pred)
    raw+=pack(202,1,120_000_000_000,1,-1,0,track)+pack(200,1,120_000_000_000,1,-1,-1,retrieval)
    raw+=pack(201,1,120_000_000_000,1,0,0,d1)+pack(203,1,120_000_000_000,1,0,0,np.r_[0,16,7,16,R,H.ravel(),z])
    raw+=pack(201,1,120_000_000_000,1,1,0,d2)+pack(203,1,120_000_000_000,1,1,0,np.r_[0,16,7,16,R,H.ravel(),z])
    raw+=pack(200,1,120_000_000_000,1,-1,-1,cov)+pack(12,1,120_000_000_000,1,-1,-1,phase_final)
    raw+=pack(201,2,121_000_000_000,1,-1,-1,[0,7,3])
    with tempfile.TemporaryDirectory(prefix='go2-vio-diagnostics-fixture-') as name:
        root=Path(name);binary=root/'records.bin';binary.write_bytes(raw)
        writer=dict(final=True,dropped=0,writer_io_failed=False)
        good=analyze(binary,root/'good',writer_stats=writer)
        assert good['algebra_checks_passed'],good['checks']
        assert good['diagnostic_capture_complete']
        assert good['record_counts'][201]==3
        tiny=np.zeros(19);tiny[0]=5e-6
        assert np.array_equal(state_plus(before,tiny)[:9],before[:9])
        # The inverse composition branch has six columns and its own state update.
        inverse_item=dict(item);inverse_item['M']=H[:,:6].T@H[:,:6]/R
        inverse_item['ht']=H[:,:6].T@z/R;inverse_item['metadata']=np.r_[1,6,0,1,16,R,1e9,.2,np.nan,1,1,16]
        inverse_solution,_=recover_solution(inverse_item)
        inverse_data=np.r_[inverse_item['metadata'],prop,before,P.ravel(),inverse_item['M'].ravel(),inverse_item['ht'],inverse_solution,state_plus(before,inverse_solution),P.ravel()]
        inverse=root/'inverse.bin';inverse.write_bytes(MAGIC+pack(201,3,120_000_000_000,1,0,0,inverse_data)+pack(203,3,120_000_000_000,1,0,0,np.r_[1,16,6,16,R,H[:,:6].ravel(),z]))
        inverse_result=analyze(inverse,root/'inverse',writer_stats=writer)
        assert inverse_result['algebra_checks_passed'],inverse_result['checks']
        broken=np.frombuffer(d1,dtype=float).copy();broken[423]+=1.
        bad=root/'bad.bin';bad.write_bytes(MAGIC+pack(201,1,120_000_000_000,1,0,0,broken)+pack(203,1,120_000_000_000,1,0,0,np.r_[0,16,7,16,R,H.ravel(),z]))
        rejected=analyze(bad,root/'bad',writer_stats=writer)
        assert not rejected['algebra_checks_passed']
        assert rejected['checks']['HtH_reconstructed_from_actual_pixels']['failed']==1
        trunc=root/'partial.bin';trunc.write_bytes(bytes(raw[:-3]))
        partial=analyze(trunc,root/'partial',allow_partial=True,writer_stats=dict(final=False,dropped=0,writer_io_failed=False))
        assert partial['source']['partial_tail'] and not partial['diagnostic_capture_complete']
    return dict(self_test='PASS',fixtures=['binary_header_and_variable_payload','accepted_state_update','source_Exp_deadband','inverse_composition_six_column_update','original_old_state_rollback','zero_track_skip','M_HTz_from_H_z','full_prior_solution_reconstruction','frame_covariance_update','bad_M_rejected','partial_tail_not_complete'],production_runtime_analyzed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run',nargs='?',type=Path);parser.add_argument('--output',type=Path);parser.add_argument('--binary',type=Path)
    parser.add_argument('--self-test',action='store_true');parser.add_argument('--allow-partial',action='store_true')
    parser.add_argument('--window',action='append',help='source seconds, start-exclusive:end-inclusive')
    args=parser.parse_args()
    if args.self_test:print(json.dumps(self_test(),indent=2));return
    if args.run is None:parser.error('RUN is required unless --self-test')
    candidates=[args.run/'fastlivo_diagnostics/records.bin',args.run/'slam_diagnostics/records.bin',args.run/'diagnostics/records.bin']
    binary=args.binary or next((p for p in candidates if p.exists()),None)
    if binary is None:parser.error('No diagnostic binary found; supply --binary explicitly')
    writer_path=binary.parent/'writer_stats.json';writer=json.loads(writer_path.read_text()) if writer_path.exists() else {}
    output=args.output or args.run/('vio_analysis_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    max_iterations=5;config_path=args.run/'navigation_fastlivo.yaml'
    if config_path.exists():
        config=json.loads(config_path.read_text());max_iterations=int(config['/**']['ros__parameters']['vio']['max_iterations'])
    windows=[tuple(map(float,window.split(':'))) for window in args.window] if args.window else None
    result=analyze(binary,output,detail_begin=float(writer.get('detail_begin_s',115)),detail_end=float(writer.get('detail_end_s',165)),windows=windows,allow_partial=args.allow_partial,writer_stats=writer,max_iterations=max_iterations)
    print(json.dumps(clean(dict(output=str(output),diagnostic_capture_complete=result['diagnostic_capture_complete'],algebra_checks_passed=result['algebra_checks_passed'],record_counts=result['record_counts'])),indent=2))


if __name__=='__main__':main()
