#!/usr/bin/env python3
"""Post-completion passive recorder metadata audit; never loads/replays XYZ NPZ."""
from pathlib import Path
import hashlib,json,datetime
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
RUN=ROOT/'runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p):return [json.loads(v) for v in Path(p).read_text().splitlines() if v.strip()]
def main():
 if(OUT/'recorder_metadata.json').exists():raise RuntimeError('Never overwrite audit receipts')
 mpath=RUN/'dynamic_sensor_evidence_manifest.json';m=json.loads(mpath.read_text());rawpath=RUN/'dynamic_sensor_evidence/sensor_inputs.jsonl';sensor=rows(rawpath)
 guards=rows(RUN/'navigation_guard_history.jsonl');first={};xyz={};pre=[]
 for v in sensor:
  if v.get('source')=='cloud':
   first.setdefault(v['stamp_ns'],v)
   if v.get('xyz_file'):xyz[v['stamp_ns']]=v
   if v.get('archive_kind')=='pre_roll':pre.append(v)
 names={v['xyz_file'] for v in sensor if v.get('xyz_file')};files=m['files'];missing=[name for name in files if not(RUN/name).is_file() or(RUN/name).stat().st_size==0]
 comparison_keys=['source','stamp_ns','received_monotonic_wall','frame_id','data_sha256','point_step','row_step','fields','width','height','is_bigendian']
 prefixmatches=all(all(v[k]==first[v['stamp_ns']][k] for k in comparison_keys) for v in pre)
 metadata_good=all(type(v['stamp_ns'])is int and v['xyz_file']=='cloud_'+str(v['stamp_ns'])+'.npz' and v['decoded_xyz_shape'][1:]==[3] and len(v['decoded_xyz_sha256'])==64 and len(v['data_sha256'])==64 and v['navigation_input'] is False for v in pre)
 allrefs=all('dynamic_sensor_evidence/'+n in files for n in names)
 referenced={v['cloud_header_stamp_ns'] for v in guards};uncovered=sorted(referenced-set(xyz));stamps=[v['stamp_ns'] for v in pre]
 checks={'manifest_schema2_no_writer_or_ring_or_observer_or_cleanup_error':m['schema']==2 and m['status']=='recorded_unverified' and m['queue_error'] is None and m['observer_error'] is None and m['cleanup_errors']==[] and m['pre_roll']['error'] is None,
 'pre_roll_flushed_and_empty_bounded_cache':m['pre_roll_flushed'] is True and m['pre_roll']['cached_frames']==0 and m['pre_roll']['cached_bytes']==0 and m['pre_roll']['peak_frames']<=m['pre_roll']['max_frames'] and m['pre_roll']['peak_bytes']<=m['pre_roll']['max_bytes'],
 'exact_prefixcount_and_source_order_within1s':len(pre)==m['pre_roll']['flushed_frames'] and len(pre)>0 and stamps==sorted(set(stamps)) and max(stamps)-min(stamps)<=1000000000,
 'prefix_original_callback_metadata_and_stamps_unmodified':prefixmatches and metadata_good and all(v['original_source_stamps_preserved'] is True and v['pre_roll_flush_monotonic_wall']>=v['received_monotonic_wall'] for v in pre),
 'all_manifest_files_present_nonempty_and_XYZ_refs_in_inventory':not missing and allrefs and len(names)==m['archived_unique_xyz']}
 x={'asof_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'checks':checks,'passed':all(checks.values()),'run':str(RUN),'source_files_hashed':{str(p.relative_to(RUN)):sha(p) for p in [mpath,rawpath,RUN/'navigation_guard_history.jsonl']},'executed_audit_sha256':sha(__file__),'recorder_manifest_without_inventory':{k:v for k,v in m.items() if k!='files'},'inventory_files':len(files),'unique_xyz':len(names),'original_pre_roll_rows':pre,'guard_source_stamp_coverage_metadata_only':{'guard_records':len(guards),'unique_required_cloud_stamps':len(referenced),'uncovered_stamps':uncovered,'prefix_stamps_consumed_by_guard':sorted(set(stamps)&referenced)},'missing_or_empty_inventory_files':missing,'scope':'Only completed-run manifest/source-row timestamp/filename/inventory metadata. No NPZ loaded, no geometry/physical/nav re-evaluation; candidate agent separately verifies decoded XYZ/filter hashes and actual guard results. A metadata match alone is not XYZ truth or complete navigation proof.','actual_physics_or_navigation_pass_assigned':False,'no_original_log_or_runtime_write':True}
 (OUT/'recorder_metadata.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps({'passed':x['passed'],'checks':checks,'prefix':stamps,'coverage_metadata_only':x['guard_source_stamp_coverage_metadata_only'],'receiptsha':sha(OUT/'recorder_metadata.json')}))
 if not x['passed']:raise SystemExit(1)
if __name__=='__main__':main()
