from pathlib import Path
import hashlib,json,shutil,subprocess,time,xml.etree.ElementTree as E
base=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_'); demo=base/'multifloor_demo'
A=demo/'development/r86_feedback_isolation/navigation_acceptance_round27_20261024'
W=A/'worktrees/published_formal22_repro'; F=W/'releases/formal22'; deps=F/'dependencies'; D=deps/'workspace/multifloor_demo'; T=D/'teacher_mode'
out=W/'reproduction/assets';out.mkdir(parents=True,exist_ok=True)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
expected={'go2_converted.sdf':'bc31d84d25cca05a58da367fb950d4385a6215f5d6dda2149f611a2f6b195924','three_floors.sdf':'1f781af572ba8ce70ff926e9789cc1a9db326e61822660fcb66a623eb3765161','go2_measured.urdf':'9ca0361f56df92550ab8588a690ae57db8dcaf8c16cf8beb170dd6eee7cf1748'}
oldmesh=base/'go2_sim_control/install/unitree_go2_description/share/unitree_go2_description'
newmesh=deps/'workspace/go2_sim_control/install/unitree_go2_description/share/unitree_go2_description'
oldrs=Path('/opt/ros/jazzy/share/realsense2_description');newrs=deps/'sdk/realsense2_description'
rows=[]
for src,dst in [(oldmesh/'meshes'/n,newmesh/'meshes'/n) for n in ['L1_4D_Lidar.dae','VLP16_base_1.dae','VLP16_base_2.dae','VLP16_scan.dae','calf.dae','hip.dae','thigh.dae','thigh_mirror.dae','trunk.dae']]+[(oldrs/'meshes/d455.stl',newrs/'meshes/d455.stl'),(oldmesh/'package.xml',newmesh/'package.xml'),(oldrs/'package.xml',newrs/'package.xml')]:
 dst.parent.mkdir(parents=True,exist_ok=True); assert not dst.exists() or sha(dst)==sha(src),(src,dst);shutil.copy2(src,dst);rows.append({'source':str(src),'destination':str(dst),'bytes':dst.stat().st_size,'source_sha256':sha(src),'destination_sha256':sha(dst),'historical_direct_sha_binding':False})
# Preserve installed upstream copyright if one is present; do not invent Unitree licensing.
for p in sorted(oldrs.glob('*')):
 if p.is_file() and any(k in p.name.lower() for k in ('license','copyright','notice')):
  shutil.copy2(p,newrs/p.name);rows.append({'source':str(p),'destination':str(newrs/p.name),'source_sha256':sha(p),'destination_sha256':sha(newrs/p.name)})
replacements={str(oldmesh/'meshes'):str(newmesh/'meshes'),'model://realsense2_description/meshes/d455.stl':(newrs/'meshes/d455.stl').as_uri(),'package://realsense2_description/meshes/d455.stl':(newrs/'meshes/d455.stl').as_uri()}
asset=[]
for n,pin in expected.items():
 src=demo/'simulation/generated'/n;assert sha(src)==pin,(n,sha(src));raw=src.read_text();new=raw
 for a,b in replacements.items():new=new.replace(a,b)
 dst=D/'simulation/generated'/n;dst.parent.mkdir(parents=True,exist_ok=True);assert not dst.exists() or dst.read_text()==new;dst.write_text(new)
 # Reversing only the declared URI substitutions must recover every source byte.
 rev=new
 for a,b in reversed([(a,b) for a,b in replacements.items() if a in raw]):rev=rev.replace(b,a)
 assert rev==raw,n
 E.parse(dst)
 asset.append({'name':n,'source':str(src),'destination':str(dst),'original_sha256':pin,'relocated_sha256':sha(dst),'bytes':dst.stat().st_size,'uri_only':True,'historical_direct_sha_binding':n!='go2_measured.urdf'})
# Every mesh URI in restored inputs must resolve within this new dependency directory.
uris=[]
for x in asset:
 r=E.parse(x['destination']).getroot()
 vals=[e.text for e in r.iter('uri')]+[e.get('filename') for e in r.iter('mesh') if e.get('filename')]
 for u in vals:
  assert u.startswith('file://'),u
  p=Path(u[7:]);assert p.is_file() and p.is_relative_to(deps),u
  uris.append(u)
(out/'UPSTREAM_RESOURCES.md').write_text('Model resources copied from the exact referenced installed packages. Unitree package.xml declares “TODO: License declaration”; no mesh license grant is inferred. Preserve upstream provenance and resolve its redistribution terms before treating the package as independently licensed. Realsense package.xml declares Apache License 2.0 (upstream intel-ros/realsense). These package.xml files are preserved next to the resources. Mesh bytes are newly explicitly pinned SDK inputs, not claimed to have individual historical formal22 scope hashes. No physical geometry, inertia, limits or sensor values changed.\n')
receipt={'schema':'published_formal22_asset_closure/v1','assets':asset,'resources':rows,'uri_replacements':replacements,'resolved_uri_count':len(uris),'URDF_boundary':'current original explicit SHA pin; output compared to retained formal22 frames, no direct historical source pin claimed','no_ROS_Gazebo_or_Actor_started':True}
(out/'ASSET_COPY_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
output=Path('/tmp/published_formal22_assets_20261025_0630');assert not output.exists(),output
cmd=['python3','-B',str(T/'simulation/prepare.py'),'--output',str(output),'--terrain','ramp_up','--sensors','--camera-rate','10','--render-engine','ogre2','--real-time-factor','1','--overview-fov','1.3962634015954636','--overview-pose','8','-1','2.5','0','.45','1.5707963267948966']
t=time.monotonic();p=subprocess.run(cmd,text=True,capture_output=True);elapsed=time.monotonic()-t
(out/'prepare.stdout.log').write_text(p.stdout);(out/'prepare.stderr.log').write_text(p.stderr)
receipt['offline_prepare']={'argv':cmd,'returncode':p.returncode,'wall_s':elapsed,'output':str(output)}
assert p.returncode==0,p.stderr
R=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261025T042130Z_closed_loop_R85_compact_formal_22_curve_grade_round27_74bc')
# Normalize only declared mesh URIs before comparing retained robot frame XML.
def normalize_tree(path):
 root=E.parse(path).getroot()
 for e in root.iter():
  if e.text:
   for a,b in replacements.items():e.text=e.text.replace(a,b)
  for k,v in list(e.attrib.items()):
   for a,b in replacements.items():v=v.replace(a,b)
   e.set(k,v)
 return E.tostring(root)
a=normalize_tree(output/'frames.urdf');b=normalize_tree(R/'frames.urdf');receipt['formal22_frames_uri_normalized_equal']=a==b
assert a==b,'formal22 frames structural mismatch'
receipt['outputs']={p.name:{'sha256':sha(p),'bytes':p.stat().st_size} for p in output.iterdir() if p.is_file()}
# Selected transport preset and actually consumed installed SDK closure, no participant created.
m=json.loads((T/'tests/cloud_transport_probe_20261004/config_env_manifest.json').read_text());checks=[]
for path,pin in m['installed_binary_sha256'].items():
 pp=Path(path);checks.append({'path':path,'exists':pp.is_file(),'sha256_match':pp.is_file() and sha(pp)==pin})
sel=m['profiles']['shm_64m']; pp=T/'tests/cloud_transport_probe_20261004'/sel['filename'];checks.append({'path':str(pp),'exists':pp.is_file(),'sha256_match':sha(pp)==sel['sha256']})
receipt['selected_transport_inputs']=checks;assert all(x['sha256_match'] for x in checks)
receipt['unselected_transport_profile_missing']=[v['filename'] for v in m['profiles'].values() if not (T/'tests/cloud_transport_probe_20261004'/v['filename']).exists()]
receipt['transport_historical_source_evidence_paths_not_runtime_inputs']=list(m.get('source_evidence',{}))
(out/'ASSET_COPY_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({'returncode':p.returncode,'wall_s':elapsed,'frames_match':a==b,'assets':len(asset),'resources':len(rows),'receipt':str(out/'ASSET_COPY_RECEIPT.json'),'receipt_sha':sha(out/'ASSET_COPY_RECEIPT.json'),'output':str(output)}))
