"""Only offline retained CDR metadata, integer header coverage, no ROS context."""
from pathlib import Path
import json,struct,collections,hashlib
ROOT=Path(__file__).resolve().parents[2]
RUN=ROOT/'simulation/test_results/20261002_classic_lowD_align_first4_candidate'
HERE=Path(__file__).resolve().parent
r=json.loads((RUN/'first_four_result.json').read_text());footer=json.loads((RUN/'actuator/observer_result.json').read_text())
intervals={'active':[r['origin_stamp_ns'],r['terminal_stamp_ns']],'fourth':[r['fourth_segment_previous_receipt_stamp_ns'],r['terminal_stamp_ns']]}
fixed={'clock':1_000_000,'imu':1_000_000,'measured_joint':1_000_000,'legacy_joint':4_000_000,'jtc':4_000_000,'truth':20_000_000}
streams=collections.defaultdict(lambda:collections.defaultdict(lambda:{'count':0,'first_ns':None,'last_ns':None,'min_ns':None,'max_ns':None,'unexpected_period_edges':[],'duplicate_backward_edges':[],'max_gap_ns':None,'max_wall_gap_ns':None}))
counts=collections.Counter();lost=[];seq_errors=[];previous_seq=None;payloadbytes=0;slam_watch=[];imu_watch=[];truth_watch=[]
with (RUN/'actuator/actuator_suffix.cdrlog').open('rb') as f:
 assert f.read(16)==b'ACTUATORSUFFIX1\n'
 while True:
  sz=f.read(8)
  if not sz:break
  assert len(sz)==8
  hn,cn=struct.unpack('<II',sz);h=json.loads(f.read(hn));f.seek(cn,1);payloadbytes+=cn
  k=h['kind'];counts[k]+=1;seq=h['sequence']
  if previous_seq is not None and seq!=previous_seq+1:seq_errors.append([previous_seq,seq])
  previous_seq=seq;ns=h.get('header_stamp_ns');ns=ns if ns is not None and ns>0 else h['clock_ns'];wall=h['wall_monotonic_ns']
  if k=='middleware_lost':lost.append(h)
  if k=='slam' and 442700000000<=ns<=443200000000:slam_watch.append(h)
  if k=='imu' and 393701000000<=ns<=393707000000:imu_watch.append(h)
  if k=='truth' and 442700000000<=ns<=443200000000:truth_watch.append(h)
  for label,(lo,hi) in intervals.items():
   if not lo<=ns<=hi:continue
   v=streams[label][k];prev=v['last_ns'];v['count']+=1
   if prev is None:v['first_ns']=ns
   else:
    gap=ns-prev;v['max_gap_ns']=max(v['max_gap_ns'] or 0,gap)
    if gap<=0:v['duplicate_backward_edges'].append([prev,ns,gap])
    if k in fixed and gap!=fixed[k]:v['unexpected_period_edges'].append([prev,ns,gap])
    v['max_wall_gap_ns']=max(v['max_wall_gap_ns'] or 0,wall-v['last_wall_ns'])
   v['last_ns']=ns;v['last_wall_ns']=wall;v['min_ns']=ns if v['min_ns'] is None else min(v['min_ns'],ns);v['max_ns']=ns if v['max_ns'] is None else max(v['max_ns'],ns)
result={'scope':'Original retained integer-header index; no original driver/native omissions are repaired with this independent observer.','original_passed':r['passed'],'intervals':intervals,'streams':{label:dict(value) for label,value in streams.items()},'all_counts':dict(counts),'all_original_sequence_errors':seq_errors,'all_payload_bytes':payloadbytes,'footer_counts_match':sum(counts.values())==footer['retained_events']==footer['total_events'],'footer_payload_bytes_match':payloadbytes==footer['payload_bytes'],'footer_overwritten_events':footer['overwritten_events'],'middleware_lost_callbacks':lost,'original_driver_missing_slam_300ms':[[a['stamp_ns'],b['stamp_ns']] for a,b in zip(r['poses'],r['poses'][1:]) if intervals['active'][0]<=a['stamp_ns']<=intervals['active'][1] and b['stamp_ns']-a['stamp_ns']>200_000_000],'independent_observer_slam_4427_4432_headers':slam_watch,'independent_observer_imu_393701_393707_headers':imu_watch,'independent_observer_truth_4427_4432_headers':truth_watch,'limits':['Header gaps are observations of this subscriber, not proof of producer omissions or native controller trigger period.','Sparse contact topics publish actual contact records; no absent contact frame is invented.','A middleware-lost event clock is receiver latest-clock association, not an exact dropped publication timestamp.','The independent complete record at a driver-missing stamp diagnoses observer coverage only and never rewrites the original result.']}
path=HERE/'oct2_classic_lowD_ALIGN_first4_all_active_header_index.json';path.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'path':str(path),'SHA256':hashlib.sha256(path.read_bytes()).hexdigest(),'counts_match':result['footer_counts_match'],'active_period_edges':{k:v['unexpected_period_edges'] for k,v in streams['active'].items() if k in fixed},'fourth_counts':{k:v['count'] for k,v in streams['fourth'].items()},'lost':lost,'slam_watch_stamps':[x['header_stamp_ns'] for x in slam_watch],'imu_watch_stamps':[x['header_stamp_ns'] for x in imu_watch]},indent=2))
