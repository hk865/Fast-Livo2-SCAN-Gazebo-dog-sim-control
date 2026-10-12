"""Read-only reconstruction/audit of saved original P1 NN evidence.

Offline recomputation checks the stored original relationship; it never replaces
the captured arrays, changes an admission or claims ordinal point identity.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import resource
import sys
import time

N=Path(__file__).resolve().parent
B=Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/engineering_v34_r38_event_any_phase')
sys.path[:0]=[str(N),str(B)]
resource.setrlimit(resource.RLIMIT_AS,(1024**3,1024**3))
resource.setrlimit(resource.RLIMIT_CPU,(30,30))
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from known_scene_adapter import pointcloud_xyz,point_records,odometry_transform

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ah(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--capture-dir',type=Path,required=True)
    ap.add_argument('--receipt',type=Path,required=True);args=ap.parse_args()
    assert not args.receipt.exists(),'exclusive new receipt required'
    started=time.monotonic();rows=[];bindings={};receipt_path=args.capture_dir/'RECEIPT.json'
    receipt=json.loads(receipt_path.read_text());bindings[str(receipt_path)]=sha(receipt_path)
    for role,item in receipt['saved'].items():
        p=Path(item['path']);gpath=p/'GROUP.json';assert sha(gpath)==item['GROUP_sha256']
        gm=json.loads(gpath.read_text());bindings[str(gpath)]=sha(gpath);messages={}
        for name,ref in gm['files'].items():
            f=p/name;assert f.stat().st_size==ref['bytes'] and sha(f)==ref['sha256'];bindings[str(f)]=sha(f)
        for name in ['raw','adapted','registered','body','mapper']:
            messages[name]=deserialize_message((p/(name+'.cdr')).read_bytes(),PointCloud2 if name in ('raw','adapted','registered') else Odometry)
            msg=messages[name];assert int(msg.header.stamp.sec)*10**9+msg.header.stamp.nanosec==gm['source_ns']
        with np.load(p/'association.npz',allow_pickle=False) as z:
            index=z['original_source_index'].copy();distance=z['original_distance_m'].copy()
            mapping=z['eligible_adapted_record_index'].copy();t=z['original_T_odom_body'].copy()
        np.testing.assert_array_equal(t,odometry_transform(messages['body']))
        np.testing.assert_array_equal(t,odometry_transform(messages['mapper']))
        xyz=pointcloud_xyz(messages['adapted']);eligible=np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)>=.3)
        np.testing.assert_array_equal(mapping,np.flatnonzero(eligible))
        local=xyz[eligible]+np.array([.2,0.,.1177]);full=pointcloud_xyz(messages['registered'])
        reconstructed=(full-t[:3,3])@t[:3,:3];meta=gm['association_evidence']
        for array,key in [(local,'original_local_body_xyz_sha256'),(full,'original_registered_odom_xyz_sha256'),(reconstructed,'original_reconstructed_body_xyz_sha256'),(index,'original_index_sha256'),(distance,'original_distance_sha256')]:assert ah(array)==meta[key],key
        tree=cKDTree(local);dd,ii=tree.query(reconstructed,workers=1)
        np.testing.assert_array_equal(index,ii);np.testing.assert_array_equal(distance,dd)
        unique,counts=np.unique(index,return_counts=True);conflicts=unique[counts>1];neighborhoods=[]
        assert len(conflicts)<=64,'bounded neighborhood report, original arrays retained separately'
        raw_records=point_records(messages['raw']);adapted_records=point_records(messages['adapted'])
        for source in conflicts:
            targets=np.flatnonzero(index==source);ds,ids=tree.query(reconstructed[targets],k=4,workers=1)
            raw_index=np.flatnonzero(raw_records==adapted_records[int(mapping[source])])
            neighborhoods.append(dict(original_source_index=int(source),adapted_record_index=int(mapping[source]),
                raw_record_matches=raw_index.tolist(),raw_beam_identity_unique=len(raw_index)==1,
                raw_beam_row_column=[int(raw_index[0]//messages['raw'].width),int(raw_index[0]%messages['raw'].width)] if len(raw_index)==1 else None,
                registered_target_indices=targets.tolist(),original_distance_m=distance[targets].tolist(),
                offline_four_nearest_source_indices=np.asarray(ids).tolist(),offline_four_nearest_distance_m=np.asarray(ds).tolist(),
                registered_reconstructed_body_xyz=reconstructed[targets].tolist(),
                original_nearest_source_body_xyz=local[int(source)].tolist(),
                ordinal_correspondence_is_not_proven=True))
        ordinal=np.linalg.norm(reconstructed-local,axis=1)
        row=dict(role=role,source_ns=gm['source_ns'],points=len(index),unique_source_indices=len(unique),
            original_collision_count=len(index)-len(unique),original_RMS_m=float(np.sqrt(np.mean(distance**2))),
            original_maximum_m=float(np.max(distance)),original_association_NN_arrays_recomputed_exactly=True,
            full_CDR_and_T_reconstructed_float64_hashes_equal=True,original_rejection=gm.get('source_join_rejection'),
            actual_anchor_metadata_present=gm.get('actual_anchor_metadata') is not None,
            original_match_control_allowed=(gm.get('original_match_result') or {}).get('control_allowed'),
            ordinal_RMS_m=float(np.sqrt(np.mean(ordinal**2))),ordinal_maximum_m=float(np.max(ordinal)),
            ordinal_geometry_is_diagnostic_only=True,collision_neighborhoods=neighborhoods,
            explicit_processing_commit_or_registered_point_ID='UNVERIFIED',
            point_time_model=gm['point_time_model'],IMU_coverage=gm['auxiliary'],
            complete_LIO_IMU_update_claimed=False)
        rows.append(row)
    out=dict(schema='R39_saved_original_association_reconstruction/v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        capture_dir=str(args.capture_dir),fixture=receipt.get('fixture') is True,passed=True,groups=rows,
        input_bindings=bindings,source_sha256={str(Path(__file__)):sha(__file__),str(N/'known_scene_adapter.py'):sha(N/'known_scene_adapter.py')},
        wall_s=time.monotonic()-started,peak_RSS_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        original_saved_relation_changed=False,control_admission_changed=False,new_online_run_started=False,
        Actor_ROS_Gazebo_GPU_started=False,navigation_PASS=False)
    args.receipt.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=True,fixture=out['fixture'],groups=[{k:r[k] for k in ['role','source_ns','points','original_collision_count','original_RMS_m','original_maximum_m','ordinal_maximum_m']} for r in rows],wall_s=out['wall_s'],receipt=str(args.receipt)),ensure_ascii=False))
if __name__=='__main__':main()
