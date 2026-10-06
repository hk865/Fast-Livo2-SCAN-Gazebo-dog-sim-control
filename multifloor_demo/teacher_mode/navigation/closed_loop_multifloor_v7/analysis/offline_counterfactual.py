#!/usr/bin/env python3
"""Frozen-state LIO single-iteration counterfactual. No matching/control replay."""
import argparse,csv,hashlib,json,struct
from pathlib import Path
import numpy as np
from analyze_lio import Layout,SCHEMA,clean,error,sha_prefix,summary

def solve(P,vec,H,w,meas):
 M=H.T@(w[:,None]*H);b=H.T@(w*meas);embed=np.zeros((19,19));embed[:6,:6]=M
 K=np.linalg.inv(embed+np.linalg.inv(P));G=K[:,:6]@M
 measurement=K[:,:6]@b;prior=vec-G@vec[:6];solution=measurement+prior
 contributions=(H@K[9,:6])*w*meas;pz_contributions=(H@K[5,:6])*w*meas
 return dict(M=M,HTz=b,K=K,G=G,measurement=measurement,prior=prior,solution=solution,vz_point=contributions,pz_point=pz_contributions)

def physical_summary(c,mask,s):
 if not np.any(mask):return {'count':0,'vz_contribution':0.,'pz_contribution':0.}
 n=c['normal'][mask];horiz=np.abs(n[:,2])>=.85;ids,inv=np.unique(c['plane_id'][mask],return_inverse=True);totals=np.bincount(inv,weights=s['vz_point']);pz=np.bincount(inv,weights=s['pz_point'])
 planes=[{'plane_id':int(ids[i]),'count':int((inv==i).sum()),'vz_contribution':float(totals[i]),'pz_contribution':float(pz[i])}for i in np.argsort(totals)]
 return {'count':int(mask.sum()),'solution19':s['solution'],'measurement19':s['measurement'],'prior19':s['prior'],'solution_pz':float(s['solution'][5]),'solution_vz':float(s['solution'][9]),'measurement_pz':float(s['measurement'][5]),'measurement_vz':float(s['measurement'][9]),'prior_pz':float(s['prior'][5]),'prior_vz':float(s['prior'][9]),'vz_contribution':float(s['vz_point'].sum()),'pz_contribution':float(s['pz_point'].sum()),'near_horizontal_vz_contribution':float(s['vz_point'][horiz].sum()),'non_horizontal_vz_contribution':float(s['vz_point'][~horiz].sum()),'most_negative_plane_groups':planes[:8],'most_positive_plane_groups':planes[-8:][::-1]}

def evidence_planes(c,sigma_num):
 ids,inv=np.unique(c['plane_id'],return_inverse=True);evidence=[];geom_mask=np.zeros(len(inv),bool)
 for j,pid in enumerate(ids):
  mask=inv==j;ii=np.flatnonzero(mask);i=ii[0];pts=c['point_world_iter'][mask];center=pts.mean(0);cov=(pts-center).T@(pts-center)/len(pts);ev,Q=np.linalg.eigh(cov);ratio=ev[1]/ev[2]if ev[2]>0 else np.nan;historical=c['plane_mid_eigenvalue'][i]/c['plane_max_eigenvalue'][i]if c['plane_max_eigenvalue'][i]>0 else np.nan;dot=abs(Q[:,0]@c['normal'][i]);thickness=np.sqrt(max(ev[0],0))
  # Retrospective diagnostic only. Require enough actual accepted points to
  # demonstrate a 2D, thin face with an almost perpendicular historical normal.
  verified=len(pts)>=6 and abs(c['normal'][i,2])>=.85 and historical<.05 and ratio>=.05 and dot<.2 and thickness<.015
  geom_mask[mask]=verified
  if verified or (len(pts)>=6 and abs(c['normal'][i,2])>=.85 and historical<.05):
   J=np.concatenate((pts-c['plane_center'][mask],-c['normal'][mask]),axis=1);candidate_plane=np.einsum('ni,nij,nj->n',J,c['plane_covariance'][mask],J);candidate_query=np.einsum('ni,nij,nj->n',c['normal'][mask],c['query_covariance'][mask],c['normal'][mask]);body_world=np.einsum('ni,nij,nj->n',c['normal'][mask],c['solver_body_world_covariance'][mask],c['normal'][mask])
   evidence.append({'plane_id':int(pid),'count':len(pts),'diagnostic_geometric_conflict':bool(verified),'historical_normal':c['normal'][i],'historical_center':c['plane_center'][i],'historical_eigenvalues':[c['plane_min_eigenvalue'][i],c['plane_mid_eigenvalue'][i],c['plane_max_eigenvalue'][i]],'historical_midmax_ratio':historical,'actual_accepted_point_covariance_eigenvalues':ev,'actual_accepted_midmax_ratio':ratio,'actual_accepted_min_eigenvector':Q[:,0],'normal_agreement_abs_dot':dot,'actual_accepted_rms_thickness_m':thickness,'actual_accepted_xyz_min':pts.min(0),'actual_accepted_xyz_max':pts.max(0),'actual_accepted_point_world':pts,'historical_signed_residual_m':summary(c['signed_point_plane_distance'][mask]),'candidate_sigma_m2':summary(c['candidate_sigma'][mask]),'candidate_sigma_plane_component_m2':summary(candidate_plane),'candidate_sigma_query_component_m2':summary(candidate_query),'absolute_residual_over_actual_gate':summary(abs(c['signed_point_plane_distance'][mask])/(sigma_num*np.sqrt(c['candidate_sigma'][mask]))),'actual_sigma_num':float(sigma_num),'solver_plane_component_m2':summary(c['solver_sigma_plane'][mask]),'solver_body_world_component_m2':summary(body_world),'solver_weight':summary(c['R_inv'][mask]),'historical_plane_normal_covariance3x3':c['plane_covariance'][i,:3,:3],'birth_stamp_s':c['plane_birth_stamp_s'][i],'update_stamp_s':c['plane_update_stamp_s'][i],'update_count':int(c['plane_update_count'][i]),'frozen':bool(c['plane_frozen'][i])})
 return geom_mask,evidence

def main():
 p=argparse.ArgumentParser();p.add_argument('--records',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--times',type=float,nargs='+',default=[132.,132.3]);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 layout=Layout(json.loads(SCHEMA.read_text()));selected=[];wanted=[round(t*1e9)for t in a.times];digest=hashlib.sha256()
 with (a.records.parent/'index.csv').open()as f:
  for row in csv.DictReader(f):
   if int(row['kind'])in [100,101]and int(row['iteration'])==0 and any(abs(int(row['stamp_ns'])-t)<=2 for t in wanted):selected.append(row)
 aggregates={};cases=[]
 with a.records.open('rb')as f:
  for row in selected:
   f.seek(int(row['offset']));raw=f.read(56);h=struct.unpack('<7Q',raw);payload=f.read(h[-1]*8);digest.update(raw);digest.update(payload);v=np.frombuffer(payload,dtype='<f8');key=(h[1],h[4])
   if h[0]==100:aggregates[key]=layout.row(100,v);continue
   d=aggregates[key];c=layout.table(101,v);H,w,meas=c['H6'],c['R_inv'],c['meas'];P,vec=d['P_before'],d['vec19'];base=solve(P,vec,H,w,meas);allmask=np.ones(len(w),bool);geom,evidence=evidence_planes(c,d['actual_sigma_num']);ratio=np.divide(c['plane_mid_eigenvalue'],c['plane_max_eigenvalue'],out=np.full(len(w),np.nan),where=c['plane_max_eigenvalue']>0);horizontal=np.abs(c['normal'][:,2])>=.85
   variants=[]
   for name,remove in [('geometry_evidenced_rank1_normal_conflict',geom)]+[(f'wide_near_horizontal_midmax_lt_{threshold}',horizontal&(ratio<threshold))for threshold in [.01,.05,.1]]:
    kept=~remove;s=solve(P,vec,H[kept],w[kept],meas[kept]);variant=physical_summary(c,kept,s);variant.update(name=name,removed_count=int(remove.sum()),removed_plane_ids=np.unique(c['plane_id'][remove]).astype(int),delta_solution_vz_vs_actual=float(s['solution'][9]-d['solution19'][9]),delta_solution_pz_vs_actual=float(s['solution'][5]-d['solution19'][5]),original_K_removed_measurement_vz=float(base['vz_point'][remove].sum()),original_K_removed_measurement_pz=float(base['pz_point'][remove].sum()),P_fixed=True,rematching=False,all_other_rows_weights_residuals_fixed=True)
    variants.append(variant)
   cases.append({'sequence':h[1],'stamp_ns':h[2],'t_s':h[2]*1e-9,'iteration':h[4],'actual_state_before':d['state_iter_before'],'actual_state_after':d['state_iter_after'],'actual_solution19':d['solution19'],'actual_vec19':vec,'actual_P_pz_vz':P[5,9],'baseline':physical_summary(c,allmask,base),'baseline_reconstruction':{k:error(base[x],d[y])for k,x,y in [('M6','M','M6'),('HTz6','HTz','HTz6'),('K1','K','K1'),('G','G','G19x6'),('solution19','solution','solution19')]},'geometric_conflict_criteria':{'minimum_accepted_points':6,'historical_abs_normal_z_ge':.85,'historical_midmax_lt':.05,'actual_accepted_midmax_ge':.05,'actual_historical_normal_abs_dot_lt':.2,'actual_accepted_rms_thickness_m_lt':.015},'historical_rank1_near_horizontal_plane_evidence':evidence,'variants':variants})
 report={'schema':'frozen_actual_lio_single_iteration_counterfactual/v1','source_records':str(a.records.resolve()),'source_schema_sha256':sha_prefix(SCHEMA,SCHEMA.stat().st_size),'source_index_sha256':sha_prefix(a.records.parent/'index.csv',(a.records.parent/'index.csv').stat().st_size),'selected_actual_record_sha256':digest.hexdigest(),'requested_times_s':a.times,'cases':cases,'equation':'M=H^T R_inv H; HTz=H^T R_inv meas; K=(embed(M)+P^-1)^-1; G=K[:,0:6]M; solution=K[:,0:6]HTz+vec-G vec[:6]','limitations':['This is a finite single iteration experiment with actual frozen P, state, vec, accepted H, R and residuals. Removed rows force K and G to be recomputed','No trajectory, covariance propagation, map rebuild, rematching or closed-loop rerun was performed; it cannot establish a navigation fix','PCA is only on currently accepted points attributed to a historical plane; the criteria are retrospective diagnostics, not a deployed gate','Low historical mid/max alone is unsafe: thin true ground scans can also be rank deficient. The geometric mask requires independent current 2D face spread and near orthogonality','The geometry-evidenced mask proves accepted point / stored normal inconsistency under current state; actual world face identity and absolute height need independent frame/SDF/truth audits','M6 combines radian and meter coordinates without characteristic length normalization. No physical conditioning conclusion from raw eigenvalues alone']}
 target=a.out/'single_iteration_counterfactual.json';target.write_text(json.dumps(clean(report),indent=2,allow_nan=False)+'\n')
 print(target)
 for q in cases:
  print(q['t_s'],'actual dvz',q['actual_solution19'][9],'dpz',q['actual_solution19'][5], 'replayerr',q['baseline_reconstruction']['solution19'])
  for v in q['variants']:print(v['name'],'removed',v['removed_count'],'dvz',v['solution_vz'],'dpz',v['solution_pz'],'delta',v['delta_solution_vz_vs_actual'])

if __name__=='__main__':main()
