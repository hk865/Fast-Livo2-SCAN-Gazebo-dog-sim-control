#!/usr/bin/env python3
"""Prepare a new physical dynamic-obstacle fixture; never start ROS/Gazebo.

Asset stage changes only moving_obstacle's initial pose before simulation.
Scope stage requires the exact three preserved V4 finite-navigation receipts.
Neither stage upgrades acceptance or historical evaluation results.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
NAV=HERE.parent
sys.path.insert(0,str(NAV))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_new(path,data):
    content=json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n'
    if path.exists():
        if path.read_text()!=content:raise RuntimeError('Refusing to replace different preparation: '+str(path))
        return
    path.write_text(content)


def eligible(run,stage):
    run=Path(run).resolve()
    if run.parent!=(ROOT/'runs').resolve():raise RuntimeError('Only a unique Teacher run is eligible')
    for name in ('telemetry.jsonl','worker_ready.json','actuator.jsonl'):
        if (run/name).exists():raise RuntimeError('Refusing to prepare an already started run')
    if stage=='asset'and (run/'navigation_scope.json').exists():raise RuntimeError('Asset must be frozen before navigation scope')
    return run


def patch_world(raw,protocol):
    root=ET.fromstring(raw);world=root.find('world')
    if world is None or world.get('name')!='teacher_demo':raise RuntimeError('Only teacher_demo is eligible')
    matches=world.findall("model[@name='moving_obstacle']")
    if len(matches)!=1:raise RuntimeError('Exactly one existing obstacle is required')
    obstacle=matches[0]
    if obstacle.findtext('static')!='true':raise RuntimeError('Expected original static collidable box')
    if [float(x)for x in obstacle.findtext('link/collision/geometry/box/size').split()]!=protocol['box_size_m']:
        raise RuntimeError('Unexpected original obstacle geometry')
    pose=protocol['initial_position_world']+[0.,0.,0.]
    text=' '.join(format(x,'.17g')for x in pose)
    pattern=r'(<model\s+name="moving_obstacle"\s*>.*?<pose>)[^<]*(</pose>)'
    updated,count=re.subn(pattern,lambda m:m.group(1)+text+m.group(2),raw,count=1,flags=re.S)
    if count!=1:raise RuntimeError('Cannot replace one bounded existing model pose')
    after=ET.fromstring(updated)
    # Compare the whole XML with only that one pose normalized. No other world,
    # robot, collision, controller, renderer or physical parameter may change.
    after.find("world/model[@name='moving_obstacle']/pose").text=obstacle.findtext('pose')
    if ET.tostring(root)!=ET.tostring(after):raise RuntimeError('World patch changed more than obstacle pose')
    return updated


def prepare_asset(run):
    run=eligible(run,'asset');protocol=json.loads((HERE/'protocol.json').read_text())
    asset=json.loads((run/'asset_manifest.json').read_text());world=run/'world.sdf'
    if asset.get('terrain')!='flat'or asset.get('spawn')!=[6,-.7,.4,0]or asset.get('sensors')is not True:
        raise RuntimeError('Dynamic scope requires the tested full-sensor safe flat spawn')
    if asset.get('exclusive_writer')!='teacher_sim::TeacherActuator':raise RuntimeError('Teacher must own all joint actuation')
    if (run/'dynamic_protocol.json').exists():
        if (run/'dynamic_protocol.json').read_bytes()!=(HERE/'protocol.json').read_bytes():raise RuntimeError('Prepared protocol differs')
        if asset.get('dynamic_fixture',{}).get('protocol_sha256')!=sha(HERE/'protocol.json')or asset.get('world_sha256')!=sha(world):
            raise RuntimeError('Previously prepared dynamic asset changed')
        return asset['dynamic_fixture']
    if asset.get('world_sha256')!=sha(world):raise RuntimeError('Generated world differs from asset receipt')
    original=world.read_text();updated=patch_world(original,protocol)
    (run/'dynamic_original_world.sdf').write_text(original)
    (run/'dynamic_original_asset_manifest.json').write_bytes((run/'asset_manifest.json').read_bytes())
    world.write_text(updated);(run/'dynamic_protocol.json').write_bytes((HERE/'protocol.json').read_bytes())
    fixture={'schema':1,'status':'prepared_unverified','experiment':protocol['experiment'],
        'allowed_entity':'moving_obstacle','original_world_sha256':sha(run/'dynamic_original_world.sdf'),
        'protocol_sha256':sha(run/'dynamic_protocol.json'),'preparer_sha256':sha(__file__),
        'changed_fields':['world/moving_obstacle/pose'],'initial_position_world':protocol['initial_position_world'],
        'policy_terrain_scan_excludes_dynamic_entity':True,'navigation_ground_truth_used':False,
        'meaning':'Physical obstacle fixture preparation only; no simulated or navigation test has run'}
    asset.update(world_sha256=sha(world),dynamic_fixture=fixture)
    temporary=run/'asset_manifest.json.tmp';temporary.write_text(json.dumps(asset,indent=2,ensure_ascii=False)+'\n');temporary.replace(run/'asset_manifest.json')
    return fixture


def validate_asset(run):
    protocol=json.loads((run/'dynamic_protocol.json').read_text())
    if (run/'dynamic_protocol.json').read_bytes()!=(HERE/'protocol.json').read_bytes():
        raise RuntimeError('Dynamic prospective protocol changed')
    asset=json.loads((run/'asset_manifest.json').read_text())
    original_world=run/'dynamic_original_world.sdf'
    original_asset=json.loads((run/'dynamic_original_asset_manifest.json').read_text())
    if sha(original_world)!=asset.get('dynamic_fixture',{}).get('original_world_sha256'):
        raise RuntimeError('Original world evidence changed')
    if sha(original_world)!=original_asset.get('world_sha256'):
        raise RuntimeError('Original generated world/manifest do not match')
    if patch_world(original_world.read_text(),protocol)!=(run/'world.sdf').read_text():
        raise RuntimeError('World differs beyond the one reviewed obstacle initial pose')
    changed=dict(asset);changed.pop('dynamic_fixture',None);changed['world_sha256']=original_asset['world_sha256']
    if changed!=original_asset:raise RuntimeError('Dynamic asset changed other physical parameters')
    return True


def basis():
    campaign=ROOT/'test_results/navigation_v4_independent_campaign/navigation_v4_independent_campaign.json'
    data=json.loads(campaign.read_text())
    if data.get('status')!='passed'or data.get('runs_evaluated')!=3 or data.get('passed_runs')!=3:
        raise RuntimeError('Exact preserved three-run V4 campaign is required')
    records=[]
    for row in data['runs']:
        path=Path(row['run_dir'])/'summary_navigation_independent.json';s=json.loads(path.read_text())
        if s.get('status')!='passed'or s.get('levels',{}).get('finite_flat_navigation')!='passed'or s.get('errors'):
            raise RuntimeError('Finite navigation basis has not passed')
        records.append({'run_id':path.parent.name,'path':str(path.resolve()),'sha256':sha(path),
                        'finite_flat_navigation':'passed','scope':'Actual finite flat SLAM/SCAN/Teacher route only'})
    if len(records)!=3 or len({x['run_id']for x in records})!=3:raise RuntimeError('Basis must contain exactly three distinct actual runs')
    return records,campaign


def runtime_files():
    return [HERE/n for n in ('dynamic_obstacle.py','prepare.py','protocol.json','flat_dynamic_profile.json','flat_dynamic_profile_turn20.json',
             'guard_trace_schema.json','obstacle_pose_observer.cpp','CMakeLists.txt','build/obstacle_pose_observer')]+[NAV/'guard_audit.py']


def prepare_scope(run):
    from scoped_profile import verify_scope,DYNAMIC_EXPERIMENTS
    run=eligible(run,'scope');parent=verify_scope(run/'navigation_scope.json')
    if parent['experiment']not in DYNAMIC_EXPERIMENTS:raise RuntimeError('Old finite flat permission excludes dynamic tests')
    data=json.loads((run/'navigation_scope.json').read_text())
    if 'dynamic_obstacle_success_claim'in data.get('excluded_scenarios',[]):raise RuntimeError('Dynamic experiment must be explicitly permitted')
    records,campaign=basis();protocol=run/'dynamic_protocol.json'
    if protocol.read_bytes()!=(HERE/'protocol.json').read_bytes():raise RuntimeError('Dynamic protocol differs from reviewed prospective criteria')
    validate_asset(run)
    paths=runtime_files()+[protocol,run/'world.sdf',run/'asset_manifest.json',run/'dynamic_original_world.sdf',
        run/'dynamic_original_asset_manifest.json',run/'navigation_scope.json',
        campaign,ROOT/'runs/acceptance.json']+[Path(row['path'])for row in records]
    scope={'schema':'teacher_dynamic_scope/v1','experiment':'finite_flat_dynamic_stop_resume_v1',
        'status':'experimental_unverified','allowed':True,'run_dir':str(run),'navigation_ground_truth_used':False,
        'navigation_experiment':parent['experiment'],'navigation_max_yaw_rate_radps':parent['profile']['max_yaw_rate_radps'],
        'global_levels_preserved':json.loads((ROOT/'runs/acceptance.json').read_text())['levels'],
        'protocol_sha256':sha(protocol),'finite_navigation_basis':records,
        'references':{str(p.resolve()):sha(p)for p in paths},
        'forbidden_outputs':json.loads(protocol.read_text())['forbidden_outputs'],
        'scope':'One physical moving_obstacle enters, blocks for10sim s and withdraws; finite SLAM route only',
        'meaning':'Explicit permission to test; never a dynamic-navigation pass or an upgrade of global acceptance'}
    save_new(run/'dynamic_scope.json',scope);return scope


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--stage',choices=('asset','scope'),required=True);args=p.parse_args()
    result=prepare_asset(args.run)if args.stage=='asset'else prepare_scope(args.run)
    print(json.dumps({'starts_ros':False,'starts_simulation':False,'stage':args.stage,'result':result},indent=2,ensure_ascii=False))


if __name__=='__main__':main()
