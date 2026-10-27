#!/usr/bin/env python3
"""External bounded raw/adapted-input ROS capture; no control publishers.

PointCloud2 CDR bytes retain the complete original message, including header,
fields, data, padding and endian information. The adapted topic is NOT claimed
to be post-FAST-LIVO preprocessing. Native telemetry supplies offline truth.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time

if not __package__:
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from evidence_runtime_v32.bounded_binary import BoundedBinaryWriter

DEFAULT_WINDOWS=((3.,4.5),(10.,12.),(208.5,215.))

def stamp_ns(msg):return int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
def selected_windows(ns,windows):return [i for i,(a,b) in enumerate(windows) if round(a*1e9)<=ns<=round(b*1e9)]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--raw-topic',required=True);p.add_argument('--adapted-topic',required=True)
    p.add_argument('--imu-topic',default='/livox/imu');p.add_argument('--clock-topic',default='/clock')
    p.add_argument('--windows-json',default=json.dumps(DEFAULT_WINDOWS))
    p.add_argument('--maximum-bytes',type=int,default=1024*1024*1024)
    p.add_argument('--duration-sim-s',type=float,default=270.)
    p.add_argument('--wall-limit-s',type=float,default=900.)
    args=p.parse_args();windows=tuple((float(a),float(b)) for a,b in json.loads(args.windows_json))
    if not windows or any(a<0 or b<a for a,b in windows):raise ValueError('Invalid bounded source-time windows')
    if len({args.raw_topic,args.adapted_topic,args.imu_topic})!=3:raise ValueError('Distinct actual sensor topics required')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2,Imu
    from rosgraph_msgs.msg import Clock
    writer=BoundedBinaryWriter(args.output/'sensor_messages.bin',maximum_file_bytes=args.maximum_bytes,
        maximum_queue_bytes=16*1024*1024,maximum_queue_records=64)
    contract=dict(schema='teacher_raw_source_window_capture/v1',run=str(args.run.resolve()),
        simulation_only=True,raw_topic=args.raw_topic,adapted_input_topic=args.adapted_topic,
        adapted_stage='after ROS sensor adapter, before FAST-LIVO preprocessing',imu_topic=args.imu_topic,
        source_time_windows_s=[list(w) for w in windows],whole_cloud_outside_windows_recorded=False,
        body_format='original ROS2 CDR serialized_message; full header/fields/data, no point conversion',
        maximum_file_bytes=args.maximum_bytes,queue_bytes=16*1024*1024,queue_records=64,
        ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),gz_partition=os.environ.get('GZ_PARTITION'),
        source_stamp_semantics='original message header stamp; no timestamp correction by recorder',
        truth_source='existing source-time native telemetry, offline comparison only; no truth subscriber',
        publications=[],navigation_ground_truth_used=False,control_or_safety_changes=False)
    (args.output/'CONTRACT.json').write_text(json.dumps(contract,indent=2)+'\n')
    rclpy.init();started=time.monotonic();stopping=[False];reason=['unknown']
    def on_signal(signum,frame):stopping[0]=True;reason[0]='signal_'+str(signum)
    signal.signal(signal.SIGTERM,on_signal);signal.signal(signal.SIGINT,on_signal)

    class Recorder(Node):
        def __init__(self):
            super().__init__('teacher_bounded_raw_evidence')
            self.clock_ns=None;self.maximum_clock_ns=None;self.sequence=0
            self.stats={name:dict(callbacks=0,eligible=0,submitted=0,serialization_failures=0,
                first_stamp_ns=None,last_stamp_ns=None,duplicate_or_reversed_stamps=0,
                per_window=[dict(eligible=0,submitted=0,first_stamp_ns=None,last_stamp_ns=None,maximum_stamp_gap_ns=0)
                    for w in windows]) for name in ('raw_cloud','adapted_input_cloud','imu')}
            qos=QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Clock,args.clock_topic,self.clock,qos)
            self.create_subscription(PointCloud2,args.raw_topic,lambda m:self.record('raw_cloud',args.raw_topic,m),qos)
            self.create_subscription(PointCloud2,args.adapted_topic,lambda m:self.record('adapted_input_cloud',args.adapted_topic,m),qos)
            self.create_subscription(Imu,args.imu_topic,lambda m:self.record('imu',args.imu_topic,m),
                QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(2.,self.save)
        def clock(self,msg):
            self.clock_ns=int(msg.clock.sec)*1_000_000_000+int(msg.clock.nanosec)
            self.maximum_clock_ns=max(self.maximum_clock_ns or 0,self.clock_ns)
            if self.clock_ns>=round(args.duration_sim_s*1e9):stopping[0]=True;reason[0]='duration_sim_clock_reached'
        def record(self,role,topic,msg):
            st=self.stats[role];st['callbacks']+=1;ns=stamp_ns(msg);ws=selected_windows(ns,windows)
            if not ws:return
            st['eligible']+=1;previous=st['last_stamp_ns']
            if previous is not None and ns<=previous:st['duplicate_or_reversed_stamps']+=1
            if st['first_stamp_ns'] is None:st['first_stamp_ns']=ns
            st['last_stamp_ns']=ns;self.sequence+=1
            meta=dict(schema='original_ros_sensor_message/v1',record_sequence=self.sequence,role=role,
                topic=topic,source_stamp_ns=ns,header_frame_id=msg.header.frame_id,
                received_monotonic_wall_ns=time.monotonic_ns(),callback_clock_ns=self.clock_ns,
                selected_window_indices=ws,navigation_ground_truth_used=False)
            if role!='imu':
                meta.update(ros_message_type='sensor_msgs/msg/PointCloud2',height=int(msg.height),width=int(msg.width),
                    is_bigendian=bool(msg.is_bigendian),point_step=int(msg.point_step),row_step=int(msg.row_step),
                    is_dense=bool(msg.is_dense),data_bytes=len(msg.data),
                    original_data_sha256=hashlib.sha256(memoryview(msg.data)).hexdigest(),
                    fields=[dict(name=f.name,offset=int(f.offset),datatype=int(f.datatype),count=int(f.count)) for f in msg.fields])
            else:meta['ros_message_type']='sensor_msgs/msg/Imu'
            try:body=serialize_message(msg);ok=writer.submit(meta,body)
            except Exception as error:
                st['serialization_failures']+=1;st['last_error']=type(error).__name__+': '+str(error);ok=False
            if ok:st['submitted']+=1
            for i in ws:
                w=st['per_window'][i];w['eligible']+=1
                if ok:w['submitted']+=1
                if w['first_stamp_ns'] is None:w['first_stamp_ns']=ns
                if w['last_stamp_ns'] is not None and ns>w['last_stamp_ns']:
                    w['maximum_stamp_gap_ns']=max(w['maximum_stamp_gap_ns'],ns-w['last_stamp_ns'])
                w['last_stamp_ns']=ns
        def graph(self):
            return {topic:[dict(node_name=p.node_name,node_namespace=p.node_namespace,topic_type=p.topic_type)
                for p in self.get_publishers_info_by_topic(topic)]
                for topic in (args.raw_topic,args.adapted_topic,args.imu_topic)}
        def snapshot(self,final=False):
            return dict(schema='teacher_raw_source_window_capture_status/v1',final=final,
                maximum_clock_ns=self.maximum_clock_ns,clock_ns=self.clock_ns,
                sensor_streams=self.stats,writer=writer.snapshot(),publisher_graph=self.graph(),
                source_windows_have_messages=all(w['eligible']>0 for st in self.stats.values() for w in st['per_window']),
                all_requested_windows_reached=self.maximum_clock_ns is not None and self.maximum_clock_ns>=round(max(b for a,b in windows)*1e9),
                termination_reason=reason[0],wall_elapsed_s=time.monotonic()-started,
                navigation_ground_truth_used=False,
                limitation='CDR bytes preserve source messages; zero writer loss does not prove zero upstream DDS/sensor loss. Compare source stamps and native/frame metadata offline.')
        def save(self):
            tmp=args.output/'STATUS.tmp';tmp.write_text(json.dumps(self.snapshot(),indent=2)+'\n');tmp.replace(args.output/'STATUS.json')
    node=Recorder()
    try:
        while rclpy.ok() and not stopping[0]:
            if time.monotonic()-started>args.wall_limit_s:
                reason[0]='wall_limit_reached';break
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        writer.close();result=node.snapshot(final=True)
        result['eligible_message_capture_complete']=bool(result['writer']['capture_complete'] and
            result['source_windows_have_messages'] and result['all_requested_windows_reached'] and
            all(st['serialization_failures']==0 for st in node.stats.values()))
        (args.output/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
