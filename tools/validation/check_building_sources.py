#!/usr/bin/env python3
"""Check independent paths, physical configuration and PCT outcomes without digests."""
from pathlib import Path
import ast, json, xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
errors=[];count=0
for source in (ROOT/'src').rglob('*.py'):
    try:ast.parse(source.read_text());count+=1
    except SyntaxError as exc:errors.append(str(source.relative_to(ROOT))+': '+str(exc))
active=['start.py','src/bringup/building.py','src/control/truth_navigation.py',
    'src/simulation/truth_worker.py','src/simulation/mesh_terrain.py','src/control/policy/student.py',
    'src/control/policy/native/worker.py','src/control/policy/native/observation.py',
    'src/control/tracking/policy_selection.py','src/planning/pct/generate_routes.py',
    'config/building/student_profile.json','assets/building/world.sdf.in']
for rel in active:
    text=(ROOT/rel).read_text()
    for marker in ('/experiment/','Ros2_fastlivo2_','legbot_map_experiment','hashlib','sha256('):
        if marker in text:errors.append(rel+': prohibited runtime dependency '+marker)
world=ET.fromstring((ROOT/'assets/building/world.sdf.in').read_text())
paths=[]
for node in world.iter():
    if node.tag=='uri' and node.text and node.text.startswith('file://'):
        expanded=Path(node.text[7:].replace('${PROJECT_ROOT}',str(ROOT))).resolve()
        if not expanded.is_relative_to(ROOT)or not expanded.is_file():errors.append('unresolved/outside asset '+str(expanded))
        paths.append(str(expanded.relative_to(ROOT)))
    filename=node.get('filename','')
    if filename.startswith('${PROJECT_ROOT}'):
        expanded=Path(filename.replace('${PROJECT_ROOT}',str(ROOT)))
        if not expanded.is_file():errors.append('missing native plugin '+str(expanded))
physics=world.find('world/physics')
if float(physics.findtext('max_step_size'))!=.005:errors.append('physics step differs')
summary=json.loads((ROOT/'build/pct_routes/PCT_route_summary.json').read_text())
lengths={}
for case in ('stairs_short','single_floor_turn','combined_floor1_inside','multi_floor2'):
    cached=json.loads((ROOT/'assets/building/routes'/f'{case}.json').read_text())
    generated=json.loads((ROOT/'build/pct_routes'/f'{case}_route.json').read_text())
    if not summary[case]['found']or cached['world_body_points']!=generated['world_body_points']:
        errors.append('PCT regenerated route differs: '+case)
    lengths[case]=generated['length_m']
if abs(lengths['multi_floor2']-85.69597066074994)>1e-6:errors.append('multi-floor length differs')
report=dict(passed=not errors,errors=errors,parsed_python_modules=count,active_paths_checked=active,
            physical_step_s=.005,asset_paths=sorted(set(paths)),regenerated_PCT_lengths_m=lengths,
            method='syntax/path checks and geometric numeric comparison; no checksum')
(ROOT/'runs').mkdir(exist_ok=True)
(ROOT/'runs/migration_source_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(passed=not errors,errors=errors,regenerated_PCT_lengths_m=lengths),ensure_ascii=False,indent=2))
raise SystemExit(0 if not errors else 2)
