#!/usr/bin/env python3
"""Read-only ray-edge diagnostic; deployment slab epsilon remains untouched."""
import argparse,dataclasses,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'policy'))
from observation import TerrainHeightMap,_Box,quaternion_rotation
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();out=a.run.resolve()
terrain=TerrainHeightMap.from_sdf(out/'fixture_world.sdf');trace=json.loads((out/'stand_original_origin_trace.json').read_text())
class StrictBox(_Box):
    def intersect(self,starts):
        origins=(starts-self.center)@self.rotation;direction=np.array([0.,0.,-1.])@self.rotation
        entering=np.full(len(starts),-np.inf);leaving=np.full(len(starts),np.inf);valid=np.ones(len(starts),dtype=bool)
        for k in range(3):
            if abs(direction[k])<1e-12:valid&=abs(origins[:,k])<=self.half_size[k]
            else:
                aa=(-self.half_size[k]-origins[:,k])/direction[k];bb=(self.half_size[k]-origins[:,k])/direction[k]
                entering=np.maximum(entering,np.minimum(aa,bb));leaving=np.minimum(leaving,np.maximum(aa,bb))
        valid&=leaving>=np.maximum(entering,0);distance=np.where(entering>=0,entering,leaving)
        return np.where(valid&(distance>=0),distance,np.inf)
strict=TerrainHeightMap([StrictBox(s.name,s.center,s.rotation,s.half_size) if isinstance(s,_Box) else s for s in terrain.shapes],terrain.source,terrain.source_sha256,terrain.excluded_models)
rows=[]
for i,r in enumerate(trace):
    hit=np.asarray(r['height_ray_hits_world']);sensor=np.asarray(r['scanner_position_world']);starts=hit.copy();starts[:,2]=sensor[2]+20
    z,_=terrain.raycast(starts);zs,_=strict.raycast(starts);recorded=np.asarray(r['height_scan_raw']);prev=trace[max(0,i-1)];ps=np.asarray(prev['scanner_position_world'])
    def yaw(row):
        x,y,z,w=row['quaternion_xyzw'];rot=quaternion_rotation([w,x,y,z]);return np.arctan2(rot[1,0],rot[0,0])
    diff=yaw(prev)-yaw(r);cy,sy=np.cos(diff),np.sin(diff);xy=(hit[:,:2]-sensor[:2])@np.array([[cy,sy],[-sy,cy]])+ps[:2];previous_starts=np.c_[xy,np.full(187,ps[2]+20)];zp,_=terrain.raycast(previous_starts)
    rows.append({'t':r['t'],'current_recorded_ray_xy_geometry_z_max_error_m':float(abs(z-hit[:,2]).max()),'current_raw_height_from_recorded_sensor_pose_max_error_m':float(abs(sensor[2]-z-.5-recorded).max()),'candidate_previous_scanner_pose_raw_height_max_error_m':float(abs(ps[2]-zp-.5-recorded).max()) if i else None,'strict_no_xy_epsilon_current_ray_z_max_error_m':float(abs(zs-hit[:,2]).max()),'mismatched_ray_count':int((abs(z-hit[:,2])>1e-5).sum()),'sensor_pose_vs_root_snapshot_xyz_max_error_m':float(abs(sensor-np.asarray(r['position'])).max()),'center_ray_y_m':float(hit[93,1])})
bad=[r for r in rows if r['mismatched_ray_count']]
result={'scope':'Read-only analysis of immutable actual Isaac hits. StrictBox is an analysis-only epsilon0 variant; deployed observation.py remains unchanged.','strict_analytic_original_max_error_m':max(r['current_recorded_ray_xy_geometry_z_max_error_m'] for r in rows),'initial_frame_max_error_m':rows[0]['current_recorded_ray_xy_geometry_z_max_error_m'],'mismatch_frames':len(bad),'mismatched_rays':sum(r['mismatched_ray_count'] for r in rows),'analysis_only_zero_xy_epsilon_max_error_m':max(r['strict_no_xy_epsilon_current_ray_z_max_error_m'] for r in rows),'all_ray_sensor_poses_match_current_root_snapshot_max_error_m':max(r['sensor_pose_vs_root_snapshot_xyz_max_error_m'] for r in rows),'interpretation':'Five initial center rows lie infinitesimally outside floor3y=0. Current sensor pose exactly matches current root snapshot. Previous pose does not repair the existing analytical1e-9edge inclusion. Removing only the XY slab epsilon in an analysis-only function resolves all ray heights to float precision, identifying a geometry-edge discrepancy rather than a stale sensor transform. This does not establish unique cause of the observed falls or alter acceptance.','mismatch_frame_details':bad,'rows':rows}
(out/'analytic_ray_boundary_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
summary_path=out/'original_origin_analysis.json';summary=json.loads(summary_path.read_text());summary['max_actual_mesh_ray_vs_exact_SDF_error_m']=result['strict_analytic_original_max_error_m'];summary['ray_boundary_diagnostic']=str(out/'analytic_ray_boundary_diagnostic.json');summary_path.write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','mismatch_frame_details')},indent=2))
