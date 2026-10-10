"""Two whole, exactly associated F2 sensor bundles. No matcher or publisher."""
from pathlib import Path
import argparse,collections,hashlib,json,math,os,signal,struct,time
import numpy as np

MIB=1024**2
GROUP_CAP=3014656
TOTAL_CAP=25165824
TARGETS=[('F2_before_drop',[15.940861967996508,4.151975435747888]),
         ('F2_after_drop',[15.98850208530142,5.506525118966972])]
TOPICS={'raw':'/demo/teacher/raw_lidar','filtered':'/demo/slam/lidar_filtered',
        'registered':'/cloud_registered_full','body':'/demo/slam/body_odom',
        'mapper':'/aft_mapped_to_init','imu':'/livox/imu','relay_imu':'/demo/teacher/slam/imu'}
def ns(msg):return int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
def sha(blob):return hashlib.sha256(blob).hexdigest()
def allocated(n):return ((n+4095)//4096)*4096
def xyz(msg):
    types={7:'f4',8:'f8'};fields={f.name:f for f in msg.fields}
    out=np.empty((msg.height*msg.width,3),dtype='<f8')
    for j,name in enumerate(('x','y','z')):
        f=fields[name]
        if f.count!=1 or f.datatype not in types:raise ValueError('Unsupported XYZ layout')
        dtype=('>' if msg.is_bigendian else '<')+types[f.datatype]
        out[:,j]=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
             offset=f.offset,strides=(msg.row_step,msg.point_step)).reshape(-1)
    if not np.isfinite(out).all():raise ValueError('Registered XYZ contains nonfinite coordinates')
    return out
def layout(msg):
    return dict(frame_id=msg.header.frame_id,height=msg.height,width=msg.width,
        point_step=msg.point_step,row_step=msg.row_step,is_bigendian=msg.is_bigendian,
        fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in msg.fields],
        data_sha256=sha(bytes(msg.data)),data_bytes=len(msg.data))
def save_new(path,data):
    with Path(path).open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--contract',type=Path,required=True);a=ap.parse_args()
    contract=json.loads(a.contract.read_text())
    assert contract['schema']=='go2_R6_bounded_eight_observation_contract/v1'
    assert contract['disk_contract']['total_allocated_and_logical_cap_bytes']==TOTAL_CAP
    out=a.run/'bounded_shadow_samples';out.mkdir()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2,Imu
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    stopping=[False]
    for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(s,lambda *_:stopping.__setitem__(0,True))
    rclpy.init()
    class Capture(Node):
        def __init__(self):
            super().__init__('go2_bounded_F2_shadow_capture')
            self.cache={k:collections.OrderedDict() for k in TOPICS}
            self.clock_ns=0;self.groups=[];self.rejections=collections.Counter();self.callbacks=collections.Counter()
            self.started=time.monotonic();self.latest_pose=None;self.last_xyz=None
            for key,topic in TOPICS.items():
                cls=PointCloud2 if key in ('raw','filtered','registered') else Imu if 'imu' in key else Odometry
                self.create_subscription(cls,topic,lambda msg,k=key:self.receive(k,msg),qos_profile_sensor_data)
            self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
            self.create_timer(.03,self.tick)
        def clock(self,msg):self.clock_ns=int(msg.clock.sec)*10**9+int(msg.clock.nanosec)
        def receive(self,key,msg):
            self.callbacks[key]+=1;stamp=ns(msg)
            c=self.cache[key];c[stamp]=(msg,time.monotonic_ns(),self.clock_ns)
            limit=128 if 'imu' in key else 12 if key in ('body','mapper') else 8
            while len(c)>limit:c.popitem(last=False)
            if key=='body':self.latest_pose=msg
        def tick(self):
            if len(self.groups)==2:stopping[0]=True;return
            if time.monotonic()-self.started>480. or self.clock_ns>=270010000000:
                stopping[0]=True;return
            label,target=TARGETS[len(self.groups)]
            for stamp,(body,_,_) in list(self.cache['body'].items()):
                pos=body.pose.pose.position
                if math.hypot(pos.x-target[0],pos.y-target[1])>.15:continue
                if self.clock_ns<stamp+50000000:continue
                if any(stamp not in self.cache[k] for k in ['raw','filtered','registered','mapper']):
                    self.rejections['missing_exact_source_bundle']+=1;continue
                if self.last_xyz and math.hypot(pos.x-self.last_xyz[0],pos.y-self.last_xyz[1])<.8:continue
                try:self.commit(label,stamp,[pos.x,pos.y,pos.z])
                except Exception as e:
                    self.rejections[type(e).__name__+': '+str(e)]+=1
                else:return
        def commit(self,label,stamp,position):
            cloud={k:self.cache[k][stamp][0] for k in ['raw','filtered','registered']}
            if cloud['raw'].header.frame_id!='velodyne' or cloud['filtered'].header.frame_id!='velodyne' or cloud['registered'].header.frame_id!='camera_init':raise ValueError('Unexpected source frame')
            payloads={'raw.cdr':bytes(serialize_message(cloud['raw'])),
                      'filtered.cdr':bytes(serialize_message(cloud['filtered']))}
            array=xyz(cloud['registered'])
            import io
            buffer=io.BytesIO();np.save(buffer,array,allow_pickle=False);payloads['registered_xyz.npy']=buffer.getvalue()
            if allocated(len(payloads['raw.cdr']))>MIB or allocated(len(payloads['filtered.cdr']))>MIB or allocated(len(payloads['registered_xyz.npy']))>768*1024:raise ValueError('Whole frame exceeds component quota')
            sync=[];coverage={}
            for key in ['imu','relay_imu']:
                records=[(t,x) for t,x in self.cache[key].items() if stamp-50000000<=t<=stamp+50000000]
                stamps=sorted(t for t,_ in records)
                if not stamps or stamps[0]>stamp-45000000 or stamps[-1]<stamp+45000000 or any(b-a>10000000 for a,b in zip(stamps,stamps[1:])):raise ValueError('Incomplete IMU support '+key)
                coverage[key]=dict(count=len(stamps),first_ns=stamps[0],last_ns=stamps[-1],maximum_gap_ns=max([b-a for a,b in zip(stamps,stamps[1:])],default=0))
                sync.extend((key,t,x) for t,x in records)
            sync.extend((key,stamp,self.cache[key][stamp]) for key in ['body','mapper'])
            sync_bytes=bytearray(b'GO2BOUNDED001LE\0');decision=time.monotonic_ns()
            for key,t,(msg,received,clock) in sync:
                blob=bytes(serialize_message(msg));meta=dict(role=key,topic=TOPICS[key],source_ns=t,
                    received_monotonic_wall_ns=received,callback_clock_ns=clock,body_bytes=len(blob),body_sha256=sha(blob),
                    arrived_before_bundle_commit=received<=decision,IMU_world_orientation_used=False)
                metadata=json.dumps(meta,sort_keys=True).encode();sync_bytes.extend(struct.pack('<QQ',len(metadata),len(blob)));sync_bytes.extend(metadata);sync_bytes.extend(blob)
            payloads['sync.bin']=bytes(sync_bytes)
            meta=dict(schema='go2_bounded_exact_source_shadow_bundle/v1',label=label,source_ns=stamp,
                selection_hint_only_frontend_position=position,selection_is_match_prior=False,
                original_header_preserved=True,integer_source_association_exact=True,
                estimator_stage_commit_sequence_certified=False,
                IMU_coverage=coverage,commit_monotonic_wall_ns=decision,
                point_time_fields=[f.name for f in cloud['raw'].fields if 'time' in f.name.lower()],
                point_time_semantics='unavailable_without_versioned_producer_snapshot_evidence',
                frontend_Z_prior_used=False,IMU_world_orientation_used=False,GT_pose_used=False,
                source_layout={k:layout(v) for k,v in cloud.items()},
                registered_original_CDR_sha256=sha(bytes(serialize_message(cloud['registered']))),
                registered_shape=list(array.shape),registered_canonical_xyz_sha256=sha(array.tobytes()),
                files={n:dict(bytes=len(v),sha256=sha(v)) for n,v in payloads.items()},
                matching_or_pose_admission=False,control_allowed=False)
            payloads['bundle.json']=(json.dumps(meta,ensure_ascii=False,indent=2)+'\n').encode()
            if allocated(len(payloads['sync.bin']))+allocated(len(payloads['bundle.json']))>128*1024:raise ValueError('Sync metadata exceeds quota')
            if 4096+sum(allocated(len(v)) for v in payloads.values())>GROUP_CAP:raise ValueError('Whole group exceeds quota')
            folder=out/label;folder.mkdir()
            for name,data in payloads.items():save_new(folder/name,data)
            self.groups.append(dict(label=label,source_ns=stamp,path=str(folder),position_hint=position,
                allocated_bytes=folder.stat().st_blocks*512+sum((folder/n).stat().st_blocks*512 for n in payloads)))
            self.last_xyz=position
    node=Capture();error=None
    try:
        while not stopping[0]:rclpy.spin_once(node,timeout_sec=.05)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except Exception as e:error=type(e).__name__+': '+str(e)
    finally:
        result=dict(schema='go2_two_F2_whole_bundle_capture_result/v1',complete=len(node.groups)==2,
            groups=node.groups,requested_groups=2,missing_groups=[x[0] for x in TARGETS[len(node.groups):]],
            rejections=dict(node.rejections),callbacks=dict(node.callbacks),error=error,
            contract_path=str(a.contract),contract_sha256=sha(a.contract.read_bytes()),
            raw_payloads_truncated=False,GT_subscriber=False,control_allowed=False,
            source_stamp_point_timing_certified=False,pose_stage_commit_certified=False)
        save_new(out/'RESULT.json',(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode())
        used=out.stat().st_blocks*512+sum(p.stat().st_blocks*512 for p in out.rglob('*'))
        if used>TOTAL_CAP:raise RuntimeError('Sampling cap violated')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result['complete'] else 1
if __name__=='__main__':raise SystemExit(main())
