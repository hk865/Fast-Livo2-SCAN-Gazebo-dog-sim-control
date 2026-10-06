#!/usr/bin/env python3
"""Offline actual ESIKF LIO evidence, no ROS/Gazebo/Actor imports or control outputs."""
import argparse, collections, csv, hashlib, json, struct
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
SCHEMA=HERE.parent/'lio_diagnostic_schema.json'
STATE_POSITION_Z=11
STATE_VELOCITY_Z=15

def sha_prefix(path,size):
 h=hashlib.sha256()
 with path.open('rb') as f:
  while size:
   b=f.read(min(size,1024*1024))
   if not b:raise ValueError('source shrank during prefix hash')
   h.update(b);size-=len(b)
 return h.hexdigest()

def clean(x):
 if isinstance(x,np.ndarray):return clean(x.tolist())
 if isinstance(x,(np.integer,np.floating)):return clean(x.item())
 if isinstance(x,float) and not np.isfinite(x):return None
 if isinstance(x,dict):return {str(k):clean(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [clean(v)for v in x]
 return x

def summary(x):
 a=np.asarray(x,dtype=float).reshape(-1);a=a[np.isfinite(a)]
 return {'count':len(a),'min':float(a.min()),'median':float(np.median(a)),'p90':float(np.quantile(a,.9)),'p99':float(np.quantile(a,.99)),'max':float(a.max())}if len(a)else {'count':0}

class Layout:
 def __init__(self,schema):self.schema=schema
 def fields(self,kind):
  s=self.schema['kinds'][str(kind)];return s.get('fields',s.get('row_fields'))
 def row(self,kind,values):
  return {q['name']:values[q['offset']:q['offset']+q['count']].reshape(q['shape'] or [])for q in self.fields(kind)}
 def table(self,kind,values):
  p=4 if kind==103 else 3;n=int(values[p-2]);width=int(values[p-1]);expected=self.schema['kinds'][str(kind)]['row_width']
  assert width==expected,(kind,width,expected)
  assert len(values)==p+n*width,(kind,len(values),p+n*width)
  t=values[p:].reshape(n,width)
  return {q['name']:t[:,q['offset']:q['offset']+q['count']].reshape([n]+(q['shape'] or []))for q in self.fields(kind)}

def records(path,live,meta):
 with path.open('rb')as f:
  assert f.read(16)==b'FLIVODIAG0001LE\0','wrong binary schema/magic'
  meta['consumed_prefix_bytes']=16;meta['partial_tail']=False
  index=path.parent/'index.csv'
  if index.exists():
   meta['index_csv_sha256']=sha_prefix(index,index.stat().st_size);digest=hashlib.sha256();used_bytes=0
   with index.open() as listing:
    for row in csv.DictReader(listing):
     if row.get('n_values') is None:
      if not live:raise ValueError('partial index tail')
      meta['partial_tail']=True;break
     offset=int(row['offset']);kind=int(row['kind']);n=int(row['n_values'])
     assert offset==meta['consumed_prefix_bytes'],'noncontiguous index offsets'
     meta['consumed_prefix_bytes']=offset+56+n*8;meta['record_counts'][str(kind)]+=1
     if kind not in (100,101,102,103):continue
     f.seek(offset);b=f.read(56);h=struct.unpack('<7Q',b);data=f.read(n*8)
     assert h[0]==kind and h[-1]==n and h[1]==int(row['sequence']) and h[2]==int(row['stamp_ns'])
     assert len(data)==n*8,'indexed record incomplete'
     digest.update(b);digest.update(data);used_bytes+=56+len(data)
     yield h,np.frombuffer(data,dtype='<f8')
   meta['selected_record_bytes']=used_bytes;meta['selected_record_sha256']=digest.hexdigest()
   if not live:assert meta['consumed_prefix_bytes']==path.stat().st_size,'index/file endpoint mismatch'
   return
  while True:
   pos=f.tell();b=f.read(56)
   if not b:break
   if len(b)!=56:
    if not live:raise ValueError('partial final header')
    meta['partial_tail']=True;break
   h=struct.unpack('<7Q',b);n=h[-1]
   if n>64*1024*1024:raise ValueError('unreasonable payload length')
   data=f.read(n*8)
   if len(data)!=n*8:
    if not live:raise ValueError('partial final payload')
    meta['partial_tail']=True;break
   meta['consumed_prefix_bytes']=f.tell();meta['record_counts'][str(h[0])]+=1
   if h[0]in (100,101,102,103):yield h,np.frombuffer(data,dtype='<f8')

def error(a,b):
 e=np.asarray(a)-np.asarray(b);return {'maxabs':float(np.max(np.abs(e)))if e.size else 0.,'relative_frobenius':float(np.linalg.norm(e)/max(np.linalg.norm(b),1e-30))}

def analyze_aggregate(h,d):
 M=d['M6'];P=d['P_before'];raw_symmetry_error=float(abs(P-P.T).max());Pe=np.linalg.eigvalsh((P+P.T)*.5);Me,Mv=np.linalg.eigh((M+M.T)*.5)
 other=[0,1,2,3,4];schur=M[5,5]-M[5,other]@np.linalg.pinv(M[np.ix_(other,other)],rcond=1e-10)@M[other,5]
 E=np.zeros((19,19));E[:6,:6]=M
 K=np.linalg.inv(E+np.linalg.inv(P));G=K[:,:6]@M
 measurement= d['K1'][:,:6]@d['HTz6'];prior=d['vec19']-d['G19x6']@d['vec19'][:6]
 solution_reconstructed=K[:,:6]@d['HTz6']+d['vec19']-G@d['vec19'][:6]
 actual= d['state_iter_after'][9:]-d['state_iter_before'][9:]
 return {'sequence':h[1],'iteration':h[4],'stamp_ns':h[2],'t_s':h[2]*1e-9,'converged':bool(d['converged']),'stop':bool(d['stop']),
  'effective_count':int(d['effective_count']),'downsampled_count':int(d['downsampled_count']),'state_propagat_z':float(d['state_propagat'][11]),'state_after_z':float(d['state_iter_after'][11]),'state_propagat_vz':float(d['state_propagat'][15]),'state_after_vz':float(d['state_iter_after'][15]),
  'solution_pz':float(d['solution19'][5]),'solution_vz':float(d['solution19'][9]),'measurement_pz':float(measurement[5]),'measurement_vz':float(measurement[9]),'prior_iteration_pz':float(prior[5]),'prior_iteration_vz':float(prior[9]),
  'P_pz_vz':float(P[5,9]),'P_pz_pz':float(P[5,5]),'P_vz_vz':float(P[9,9]),'P_vz_bias_acc_xyz':P[9,13:16].tolist(),'P_vz_gravity_xyz':P[9,16:19].tolist(),
  'P_symmetry_maxabs':raw_symmetry_error,'P_eigenvalues_symmetrized':Pe.tolist(),'M6_eigenvalues_unscaled':Me.tolist(),'M6_weakest_eigenvector_unscaled':Mv[:,0].tolist(),'M6_rank_relative1e10':int(np.sum(Me>max(abs(Me).max(),1e-30)*1e-10)),'vertical_information_M55':float(M[5,5]),'vertical_schur_info_conditioned_other5_rcond1e10':float(schur),
  'reconstruction':{'K1':error(K,d['K1']),'G':error(G,d['G19x6']),'solution':error(solution_reconstructed,d['solution19']),'nonrotation_actual_state_delta':error(actual,d['solution19'][3:])},
  'query_summary':{k:float(d[k])for k in ['primary_exists_count','primary_matched_count','neighbor_attempted_count','actual_neighbor_exists_count','counterfactual_neighbor_exists_count','neighbor_key_mismatch_count','unmatched_count','selected_primary_count','selected_neighbor_count','radius_rejections','sigma_rejections','gate_passes','nonfinite_range_count','nonfinite_candidate_sigma_count']},'iteration_to_pre_emit_wall_s':float(d['pre_emit_omp_monotonic_s']-d['iteration_begin_omp_monotonic_s'])}

def analyze_constraints(h,c,d):
 H=c['H6'];w=c['R_inv'];z=c['meas'];n=c['normal'];count=len(w)
 finite=np.isfinite(H).all(1)&np.isfinite(w)&np.isfinite(z)&(w>0)
 if not finite.all():return {'sequence':h[1],'iteration':h[4],'stamp_ns':h[2],'invalid_rows':int((~finite).sum()),'count':count}
 M=H.T@(w[:,None]*H);HTz=H.T@(w*z)
 vz=(H@d['K1'][9,:6])*w*z;pz=(H@d['K1'][5,:6])*w*z
 floor=np.abs(n[:,2])>=.85;wall=np.abs(n[:,2])<=.20;slope=(np.abs(n[:,2])>.20)&(~floor)
 frozen=c['plane_frozen'].astype(bool);age=h[2]*1e-9-c['plane_birth_stamp_s'];point=c['point_world_iter'];center=c['plane_center'];planeCov=c['plane_covariance'];queryCov=c['query_covariance']
 J=np.concatenate((point-center,-n),axis=1);sigma_plane=np.einsum('ni,nij,nj->n',J,planeCov,J);sigma_candidate=sigma_plane+np.einsum('ni,nij,nj->n',n,queryCov,n)
 sigma_solver=c['solver_sigma_plane']+np.einsum('ni,nij,nj->n',n,c['solver_body_world_covariance'],n)
 weight_rebuilt=1/(.001+sigma_solver)
 plane_z=np.full(count,np.nan);mask=np.abs(n[:,2])>=.2;body_xy=d['state_iter_before'][9:11]
 plane_z[mask]=(-c['plane_d'][mask]-n[mask,0]*body_xy[0]-n[mask,1]*body_xy[1])/n[mask,2]
 def group(mask):
  return {'count':int(mask.sum()),'weighted_fraction':float(w[mask].sum()/max(w.sum(),1e-30)),'vz_measurement_contribution':float(vz[mask].sum()),'pz_measurement_contribution':float(pz[mask].sum()),'negative_vz_sum':float(vz[mask&(vz<0)].sum()),'positive_vz_sum':float(vz[mask&(vz>0)].sum()),'plane_z_at_body_xy':summary(plane_z[mask]),'plane_age_s':summary(age[mask]),'candidate_sigma':summary(c['candidate_sigma'][mask]),'solver_weight':summary(w[mask]),'absolute_residual_m':summary(abs(c['signed_point_plane_distance'][mask]))}
 groups={name:group(mask)for name,mask in [('near_horizontal_absnz_ge085',floor),('vertical_absnz_le020',wall),('sloped_or_other',slope),('frozen',frozen),('not_frozen',~frozen),('born_within_1s',age<=1),('primary',c['origin']==1),('actual_neighbor',c['origin']==2)]}
 def influential(indices):
  out=[]
  for i in indices:
   out.append({'source_index':int(c['source_index'][i]),'plane_id':int(c['plane_id'][i]),'origin':int(c['origin'][i]),'normal':n[i].tolist(),'center':center[i].tolist(),'plane_d':float(c['plane_d'][i]),'plane_z_at_body_xy':float(plane_z[i]),'point_world':point[i].tolist(),'signed_residual':float(c['signed_point_plane_distance'][i]),'weight':float(w[i]),'vz_measurement_contribution':float(vz[i]),'pz_measurement_contribution':float(pz[i]),'age_s':float(age[i]),'frozen':bool(frozen[i]),'candidate_sigma':float(c['candidate_sigma'][i]),'solver_sigma_plane':float(c['solver_sigma_plane'][i])})
  return out
 planes=[]
 pids,inverse=np.unique(c['plane_id'],return_inverse=True);totals=np.bincount(inverse,weights=vz)
 order=np.argsort(totals);chosen=np.unique(np.r_[order[:8],order[-8:]])
 for j in chosen:
  pid=pids[j];pm=inverse==j;planes.append({'plane_id':int(pid),'count':int(pm.sum()),'vz_measurement_contribution':float(vz[pm].sum()),'pz_measurement_contribution':float(pz[pm].sum()),'plane_z_at_body_xy':summary(plane_z[pm]),'age_s':summary(age[pm]),'frozen_fraction':float(frozen[pm].mean()),'normal_mean':n[pm].mean(0).tolist()})
 planes.sort(key=lambda x:x['vz_measurement_contribution'])
 return {'sequence':h[1],'iteration':h[4],'stamp_ns':h[2],'t_s':h[2]*1e-9,'count':count,'weighted_residual_rms_m':float(np.sqrt(np.sum(w*z*z)/w.sum()))if count else None,'normal_abs_z':summary(abs(n[:,2])),'normal_second_moment_weighted':((n.T@(w[:,None]*n))/max(w.sum(),1e-30)).tolist(),'vz_point_sum':float(vz.sum()),'pz_point_sum':float(pz.sum()),'groups':groups,'reconstruction':{'M6':error(M,d['M6']),'HTz6':error(HTz,d['HTz6']),'candidate_sigma':error(sigma_candidate,c['candidate_sigma']),'solver_weight':error(weight_rebuilt,w),'vz_point_sum_vs_K_HTz':error(np.array(vz.sum()),np.array((d['K1'][9,:6]@d['HTz6'])))},'most_negative_points':influential(np.argsort(vz)[:8]),'most_positive_points':influential(np.argsort(vz)[-8:][::-1]),'most_negative_planes':planes[:8],'most_positive_planes':planes[-8:][::-1]}

def analyze_queries(h,q):
 attempted=q['neighbor_attempted'].astype(bool);mismatch=q['neighbor_key_mismatch'].astype(bool);matched=q['final_matched'].astype(bool)
 return {'sequence':h[1],'iteration':h[4],'stamp_ns':h[2],'t_s':h[2]*1e-9,'point_count':len(matched),'primary_absent_count':int((q['primary_exists']==0).sum()),'matched_count':int(matched.sum()),'fallback_attempted_count':int(attempted.sum()),'fallback_actual_matched_count':int((q['selected_origin']==2).sum()),'fallback_key_mismatch_count':int((attempted&mismatch).sum()),'counterfactual_exists_actual_absent_count':int((attempted&(q['actual_neighbor_exists']==0)&(q['counterfactual_neighbor_exists']!=0)).sum()),'mismatched_fallback_matched_count':int((mismatch&(q['selected_origin']==2)).sum()),'unmatched_with_any_radius_rejection':int(((~matched)&(q['radius_rejections']>0)).sum()),'unmatched_with_any_sigma_rejection':int(((~matched)&(q['sigma_rejections']>0)).sum()),'unmatched_with_no_plane_nodes':int(((~matched)&(q['visited_plane_nodes']==0)).sum()),'examples_key_mismatch':[{'source_index':int(q['source_index'][i]),'point_world':q['point_world_iter'][i].tolist(),'primary_key':q['primary_voxel_key'][i].tolist(),'actual_neighbor_key':q['actual_neighbor_voxel_key'][i].tolist(),'counterfactual_neighbor_key':q['counterfactual_neighbor_voxel_key'][i].tolist(),'matched':bool(matched[i]),'plane_id':int(q['selected_plane_id'][i])}for i in np.flatnonzero(mismatch)[:8]],'counterfactual_matching_run':False}

def main():
 p=argparse.ArgumentParser();p.add_argument('--records',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--schema',type=Path,default=SCHEMA);p.add_argument('--live',action='store_true');args=p.parse_args()
 args.out.mkdir(parents=True,exist_ok=True);schema=json.loads(args.schema.read_text());layout=Layout(schema);meta={'record_counts':collections.Counter()};aggregates={};agg_rows=[];constraints=[];queries=[];map_events=[]
 for h,a in records(args.records,args.live,meta):
  kind=h[0]
  if kind==100:
   assert len(a)==schema['kinds']['100']['payload_width'];d=layout.row(kind,a);aggregates[(h[1],h[4])]=d;agg_rows.append(analyze_aggregate(h,d))
  elif kind==101:
   d=aggregates.get((h[1],h[4]));assert d is not None,'detail has no preceding aggregate';constraints.append(analyze_constraints(h,layout.table(kind,a),d))
  elif kind==102:queries.append(analyze_queries(h,layout.table(kind,a)))
  elif kind==103:
   d=layout.table(kind,a)
   for i in range(len(d['plane_id'])):
    map_events.append({'sequence':h[1],'stamp_ns':h[2],'t_s':h[2]*1e-9,'event':int(d['event_code'][i]),'plane_id':int(d['plane_id'][i]),'is_plane':bool(d['is_plane'][i]),'is_init':bool(d['is_init'][i]),'tree_layer':int(d['tree_layer'][i]),'voxel_center':d['voxel_center_m'][i].tolist(),'normal':d['normal'][i].tolist(),'center':d['center'][i].tolist(),'d':float(d['d'][i]),'point_count':int(d['plane_point_count'][i]),'birth_stamp_s':float(d['plane_birth_stamp_s'][i]),'age_s':float(d['plane_age_s'][i]),'update_count':int(d['plane_update_count'][i]),'update_enabled':bool(d['update_enabled'][i]),'min_eigenvalue':float(d['min_eigenvalue'][i])})
 stats_path=args.records.parent/'writer_stats.json';stats=json.loads(stats_path.read_text())if stats_path.exists()else None
 errors=[row['reconstruction']for row in agg_rows];summary_report={'schema':'actual_lio_solver_fault_analysis/v1','source_records':str(args.records.resolve()),'source_schema':str(args.schema.resolve()),'schema_sha256':sha_prefix(args.schema,args.schema.stat().st_size),'source_prefix_sha256':sha_prefix(args.records,meta['consumed_prefix_bytes']) if 'selected_record_sha256' not in meta else None,'selected_record_sha256':meta.get('selected_record_sha256'),'selected_record_bytes':meta.get('selected_record_bytes'),'index_csv_sha256':meta.get('index_csv_sha256'),'source_prefix_bytes':meta['consumed_prefix_bytes'],'live_snapshot':args.live,'partial_tail':meta['partial_tail'],'writer_stats':stats,'record_counts':dict(meta['record_counts']),'time_range_s':[min(x['t_s']for x in agg_rows),max(x['t_s']for x in agg_rows)]if agg_rows else None,'LIO_iterations':len(agg_rows),'detail_iterations':len(constraints),'map_plane_events':len(map_events),'solution_reconstruction_maxabs':max((x['solution']['maxabs']for x in errors),default=None),'state_tail_update_reconstruction_maxabs':max((x['nonrotation_actual_state_delta']['maxabs']for x in errors),default=None),'detail_M6_reconstruction_maxabs':max((x['reconstruction']['M6']['maxabs']for x in constraints if'reconstruction'in x),default=None),'detail_HTz_reconstruction_maxabs':max((x['reconstruction']['HTz6']['maxabs']for x in constraints if'reconstruction'in x),default=None),'most_negative_LIO_iteration_solution_vz':sorted(agg_rows,key=lambda x:x['solution_vz'])[:12],'most_negative_point_measurement_detail_iterations':sorted(constraints,key=lambda x:x.get('vz_point_sum',0))[:12],'neighbor_attempted_detail_points':sum(x['fallback_attempted_count']for x in queries),'neighbor_key_mismatch_detail_points':sum(x['fallback_key_mismatch_count']for x in queries),'mismatched_neighbor_matched_detail_points':sum(x['mismatched_fallback_matched_count']for x in queries),'scope':['Exact saved solver equations and actual accepted constraints; simulation native/Gazebo truth absent from this script','M6 eigenvalues mix rotation and translation units: no characteristic-length normalization, rank/Schur diagnostic proxy only','Counterfactual neighbor is only existence lookup; no matched AB or effectiveness proof','Per-point measurement contribution separated from prior/iteration term; do not label full solution as residual effect','Plane height evaluated at current body XY can extrapolate beyond a local patch; not independent absolute floor truth','Detail records first/terminal actual iteration only, intermediate correspondence rows are absent','This fault diagnosis does not change original navigation acceptance or claim physical/SLAM success']}
 for name,value in [('summary',summary_report),('aggregate_iterations',agg_rows),('constraint_details',constraints),('query_details',queries),('map_plane_events',map_events)]:
  (args.out/(name+'.json')).write_text(json.dumps(clean(value),indent=2,allow_nan=False)+'\n')
 print(json.dumps(clean({k:summary_report[k]for k in ['time_range_s','LIO_iterations','detail_iterations','map_plane_events','solution_reconstruction_maxabs','detail_M6_reconstruction_maxabs','neighbor_attempted_detail_points','neighbor_key_mismatch_detail_points','mismatched_neighbor_matched_detail_points']}),indent=2))

if __name__=='__main__':main()
