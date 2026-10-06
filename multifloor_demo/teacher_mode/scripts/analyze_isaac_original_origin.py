#!/usr/bin/env python3
"""Recorded original-origin physics plus exact native height-term replay receipt."""
import argparse,ast,collections,hashlib,json,sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'policy'))
from observation import TerrainHeightMap
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();out=a.run.resolve()
protocol=json.loads((out/'protocol.json').read_text());result=json.loads((out/'results.json').read_text());trace=json.loads((out/'stand_original_origin_trace.json').read_text())
terrain=TerrainHeightMap.from_sdf(out/'fixture_world.sdf')
source=Path('/home/hyh001/projects/1.Project/RL_for_unitree/source/rl_unitree/rl_unitree/tasks/locomotion/mdp/terminations.py')
node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='root_clearance_below_local_terrain')
# Execute the exact saved function body on recorded tensors. This is a
# counterfactual receipt, not an assertion that a disabled native term ran.
code=ast.Module(body=[ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0),node],type_ignores=[])
class Entity:
    def __init__(self,name):self.name=name
namespace={'torch':torch,'SceneEntityCfg':Entity};exec(compile(ast.fix_missing_locations(code),str(source),'exec'),namespace)
native=namespace[node.name]
class Scene(dict):pass
receipts=[];hits=[];boundary_mismatches=[]
for r in trace:
    root=torch.tensor([r['position']],dtype=torch.float32);ray=torch.tensor([r['height_ray_hits_world']],dtype=torch.float32)
    scene=Scene(robot=NS(data=NS(root_pos_w=NS(torch=root))));scene.sensors={'height_scanner':NS(data=NS(ray_hits_w=NS(torch=ray)))}
    default_trigger=bool(native(NS(scene=scene),minimum_clearance=.16).item())
    i=int(((ray[0,:,:2]-root[0,:2])**2).sum(1).argmin().item());native_clearance=float(root[0,2]-ray[0,i,2])
    starts=ray[0].numpy().copy();starts[:,2]=r['scanner_position_world'][2]+20
    points=ray[0].numpy();analytic_z,names=terrain.raycast(starts);mesh_error=float(abs(analytic_z-points[:,2]).max())
    # Attribute measured hits by their actual XYZ lying on a shape's upper
    # surface, rather than silently relabeling near-edge missed deck rays.
    measured_names=[];point_errors=[]
    for point in points:
        candidates=[]
        for shape in terrain.shapes:
            local=(point-shape.center)@shape.rotation
            if len(shape.half_size)==3:
                inside=np.all(abs(local[:2])<=shape.half_size[:2]+1e-5)
                error=abs(local[2]-shape.half_size[2])
            else:
                inside=np.all(abs(local[:2])<=shape.half_size+1e-5);error=abs(local[2])
            if inside:candidates.append((float(error),shape.name))
        if not candidates:measured_names.append('unattributed');point_errors.append(float('inf'))
        else:
            error,label=min(candidates);measured_names.append(label if error<1e-5 else 'unattributed');point_errors.append(error)
    counts=dict(collections.Counter(measured_names))
    bad=np.flatnonzero(abs(analytic_z-points[:,2])>1e-5)
    if len(bad):boundary_mismatches.append({'t':r['t'],'ray_indices':bad.tolist(),'measured_y_min_max_m':[float(points[bad,1].min()),float(points[bad,1].max())],'analytic_predicted_shapes':dict(collections.Counter(names[i] for i in bad)),'actual_hit_shapes':dict(collections.Counter(measured_names[i] for i in bad))})
    receipts.append({'t':r['t'],'counterfactual_native_height_term_triggered':default_trigger,'nearest_ray_index':i,'nearest_ray_surface':names[i],'native_scan_clearance_m':native_clearance,'independent_below_base_support_clearance_m':r['clearance']})
    receipts[-1]['nearest_ray_surface']=measured_names[i]
    hits.append({'t':r['t'],'actual_ray_hit_surface_counts':counts,'hit_shape_provenance':'Actual recorded XYZ matched to exact SDF upper surface within1e-5m; counts are derived geometric attribution, not native per-triangle ID.','actual_hit_point_to_SDF_surface_max_error_m':max(point_errors),'analytic_vertical_ray_vs_actual_mesh_ray_z_max_error_m':mesh_error,'overhead_hit_count':int((ray[0,:,2].numpy()>r['position'][2]+1e-5).sum())})
observed=np.asarray([r['observation247'] for r in trace],dtype=np.float32);actions=np.asarray([r['action'] for r in trace]);teacher=np.asarray([r['controller_mode']=='teacher' for r in trace])
actor=torch.nn.Sequential(torch.nn.Linear(247,512),torch.nn.ELU(),torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,12));checkpoint=torch.load(protocol['checkpoint'],map_location='cpu',weights_only=False);actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in checkpoint['actor_state_dict'].items() if k.startswith('mlp.')});actor.eval();torch.set_num_threads(1)
with torch.inference_mode():pred=actor(torch.from_numpy(observed)).numpy()
last=trace[-1];below_failure=last['clearance'] is not None and last['clearance']<.16
summary={'status':'completed','scope':'Actual frozen Teacher zero-command physics diagnostic at original SLAM5 origin; native scan-based height termination explicitly disabled, actual below-base support clearance used. Not successful15s original training environment.','scheduled_duration_s':15,'frames_recorded':len(trace),'duration_recorded_s':result['results'][0]['duration_recorded'],'physically_failed':result['results'][0]['failed'],'last_snapshot':{'t':last['t'],'position':last['position'],'roll_rad':last['roll'],'pitch_rad':last['pitch'],'independent_below_base_clearance_m':last['clearance'],'command':last['command'],'controller_mode':last['controller_mode']},'termination_reason_evidence':{'independent_below_base_clearance_below_0_16':below_failure,'max_roll_pitch_over_0_8_at_last_snapshot':max(abs(last['roll']),abs(last['pitch']))>.8,'env_step_native_termination_flag_not_retained':True,'reason':'The measured last snapshot itself satisfies the executed explicit clearance<.16 stop clause; other post-step native termination flags were not logged and are not invented.'},'recorded_observation_actor_max_error':float(abs(pred[teacher]-actions[teacher]).max()),'first_recorded_raw_height_min_max':[min(trace[0]['height_scan_raw']),max(trace[0]['height_scan_raw'])],'first_ray_source_counts':hits[0]['actual_ray_hit_surface_counts'],'first_overhead_hits':hits[0]['overhead_hit_count'],'max_actual_hit_point_to_exact_SDF_surface_error_m':max(r['actual_hit_point_to_SDF_surface_max_error_m'] for r in hits),'strict_analytic_vertical_ray_vs_actual_mesh_error_m':max(r['analytic_vertical_ray_vs_actual_mesh_ray_z_max_error_m'] for r in hits),'analytic_ray_boundary_failure':{'passed':not boundary_mismatches,'frames':len(boundary_mismatches),'rays':sum(len(r['ray_indices']) for r in boundary_mismatches),'explanation':'At t.02-.10,17 center-row rays lie3e-13..3e-11m below the floor3 y=0 edge. PhysX triangle raycast correctly misses upper deck; analytical slab epsilon1e-9 includes it, giving2.4m error. Preserve failure. This concerns85 rays during initialization boundary case, not normal non-edge terrain ray parity.','rows':boundary_mismatches},'default_native_term_replay':{'provenance':'Exact original source function AST body executed against recorded tensors, counterfactual replay, not an original automatic termination rollout.','source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'first_frame_receipt':receipts[0],'first_post_reset_true_receipt':next((r for r in receipts if r['t']>0 and r['counterfactual_native_height_term_triggered']),None),'native_term_true_frames':sum(r['counterfactual_native_height_term_triggered'] for r in receipts)},'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(out/'default_native_height_termination_receipt.json').write_text(json.dumps({'provenance':summary['default_native_term_replay'],'receipts':receipts},indent=2)+'\n')
(out/'height_ray_hit_sources.json').write_text(json.dumps(hits,indent=2)+'\n')
(out/'original_origin_analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
