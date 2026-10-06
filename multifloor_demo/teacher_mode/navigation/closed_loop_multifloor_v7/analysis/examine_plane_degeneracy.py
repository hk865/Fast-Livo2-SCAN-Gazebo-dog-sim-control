#!/usr/bin/env python3
"""Targeted indexed evidence for line-like historical planes; no changed matcher."""
import argparse,csv,json,struct
from pathlib import Path
import numpy as np
from analyze_lio import Layout,SCHEMA,clean,summary,sha_prefix

def main():
 p=argparse.ArgumentParser();p.add_argument('--records',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
 targets=[115,120,125,127.9,129,130,131,131.8,132,132.4,133,134,135,140,150,156.4,156.5,165];times=[round(t*1e9)for t in targets];layout=Layout(json.loads(SCHEMA.read_text()));selected=[];agg={};cases=[]
 with (args.records.parent/'index.csv').open()as f:
  for row in csv.DictReader(f):
   if int(row['kind']) not in [100,101] or int(row['iteration'])!=0:continue
   if any(abs(int(row['stamp_ns'])-t)<=2 for t in times):selected.append(row)
 with args.records.open('rb')as f:
  for row in selected:
   f.seek(int(row['offset']));h=struct.unpack('<7Q',f.read(56));a=np.frombuffer(f.read(h[-1]*8),dtype='<f8');key=(h[1],h[4])
   if h[0]==100:agg[key]=layout.row(100,a);continue
   d=agg[key];c=layout.table(101,a);H=c['H6'];w=c['R_inv'];meas=c['meas'];vz=(H@d['K1'][9,:6])*w*meas;pz=(H@d['K1'][5,:6])*w*meas
   lmin=c['plane_min_eigenvalue'];lmid=c['plane_mid_eigenvalue'];lmax=c['plane_max_eigenvalue'];ratio=np.divide(lmid,lmax,out=np.full(len(lmid),np.nan),where=lmax>0)
   horizontal=np.abs(c['normal'][:,2])>=.85
   groups={}
   for thresh in [.01,.05,.1]:
    for name,mask in [('all',ratio<thresh),('horizontal',horizontal&(ratio<thresh))]:
     groups[name+'_midmax_lt_'+str(thresh)]={'count':int(mask.sum()),'vz_contribution':float(vz[mask].sum()),'pz_contribution':float(pz[mask].sum()),'absolute_vz_contribution_sum':float(abs(vz[mask]).sum()),'weight_fraction':float(w[mask].sum()/w.sum())}
   plane_cases=[];ids,inverse=np.unique(c['plane_id'],return_inverse=True);totals=np.bincount(inverse,weights=vz);order=np.argsort(totals);chosen=np.unique(np.r_[order[:12],order[-8:]])
   for j in chosen:
    mask=inverse==j;pts=c['point_world_iter'][mask];normals=c['normal'][mask];count=len(pts);center=pts.mean(0);cov=(pts-center).T@(pts-center)/count;vals,vecs=np.linalg.eigh(cov);true_normal=vecs[:,0]
    i=np.flatnonzero(mask)[0]
    plane_cases.append({'plane_id':int(ids[j]),'count':count,'vz_contribution':float(vz[mask].sum()),'pz_contribution':float(pz[mask].sum()),'historical_center':c['plane_center'][i].tolist(),'historical_normal':c['normal'][i].tolist(),'historical_eigenvalues':[float(lmin[i]),float(lmid[i]),float(lmax[i])],'historical_midmax_ratio':float(ratio[i]),'birth_stamp_s':float(c['plane_birth_stamp_s'][i]),'frozen':bool(c['plane_frozen'][i]),'current_accepted_xyz_min':pts.min(0).tolist(),'current_accepted_xyz_max':pts.max(0).tolist(),'current_group_covariance_eigenvalues':vals.tolist(),'current_group_midmax_ratio':float(vals[1]/vals[2])if vals[2]>0 else None,'current_group_smallest_eigenvector':true_normal.tolist(),'normal_agreement_abs_dot':float(abs(true_normal@c['normal'][i])),'current_group_signed_historical_residual_m':summary(c['signed_point_plane_distance'][mask]),'candidate_sigma':summary(c['candidate_sigma'][mask]),'solver_weight':summary(w[mask]),'max_plane_center_distance_m':float(np.linalg.norm(pts-c['plane_center'][i],axis=1).max())})
   cases.append({'sequence':h[1],'stamp_ns':h[2],'t_s':h[2]*1e-9,'count':len(w),'vz_total':float(vz.sum()),'pz_total':float(pz.sum()),'horizontal_count':int(horizontal.sum()),'horizontal_vz_total':float(vz[horizontal].sum()),'historical_midmax_ratio':summary(ratio),'historical_horizontal_midmax_ratio':summary(ratio[horizontal]),'ratio_groups':groups,'planes':plane_cases})
 report={'schema':'historical_plane_degeneracy_targeted_evidence/v1','source_records':str(args.records.resolve()),'source_index_sha256':sha_prefix(args.records.parent/'index.csv',(args.records.parent/'index.csv').stat().st_size),'target_times_s':targets,'cases':cases,'limitations':['Historical covariance eigenvalues are actual saved init_plane fields','Current group PCA uses only accepted points of that historical plane, not all raw points; few/line-like groups cannot establish a unique normal','Eigen ratios are descriptive diagnostic thresholds .01/.05/.1, not proposed production gates or retroactive acceptance changes','No correspondence AB was executed; ratio contribution decomposition alone does not prove a fix','No Gazebo truth input; source physical face identity needs an independent world-geometry/offline alignment check']}
 (args.out/'plane_degeneracy.json').write_text(json.dumps(clean(report),indent=2,allow_nan=False)+'\n');print('cases',len(cases))
 for c in cases:
  if abs(c['t_s']-132)<1e-6:
   print('132 ratio groups',c['ratio_groups']);print('132 negative plane first5',c['planes'][:5])
if __name__=='__main__':main()
