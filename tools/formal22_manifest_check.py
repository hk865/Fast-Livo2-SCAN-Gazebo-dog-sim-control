#!/usr/bin/env python3
"""Read-only, standard-library check of the public formal22 source snapshot.
No module imports from the snapshot; no ROS, model, build or simulator startup.
"""
from pathlib import Path
import ast,hashlib,json,re,sys
ROOT=Path(__file__).resolve().parents[1]
RELEASE=ROOT/'releases/formal22'
manifest=json.loads((RELEASE/'SOURCE_EXPORT.json').read_text())
errors=[];counts={'hashed_files':0,'python_syntax':0,'json_files':0,'markdown_links':0}
for name,row in manifest['files'].items():
 p=RELEASE/name
 if not p.is_file():errors.append([name,'missing']);continue
 if hashlib.sha256(p.read_bytes()).hexdigest()!=row['published_sha256']:errors.append([name,'SHA mismatch'])
 counts['hashed_files']+=1
 try:
  if p.suffix=='.py':ast.parse(p.read_text(),filename=name);counts['python_syntax']+=1
  elif p.suffix=='.json':json.loads(p.read_text());counts['json_files']+=1
 except (SyntaxError,ValueError)as e:errors.append([name,type(e).__name__,str(e)[:180]])
paths=[ROOT/'README.md',*sorted((ROOT/'docs').glob('*FORMAL22*.md')),ROOT/'docs/EXPERIMENT_HISTORY.md',RELEASE/'README.md']
paths+=list(RELEASE.rglob('*.md'))
for p in sorted(set(paths)):
 for target in re.findall(r'!?\[[^\]\n]+\]\(([^)\n]+)\)',p.read_text()):
  target=target.strip('<>')
  if re.match(r'^[A-Za-z][A-Za-z0-9+.-]*:',target)or target.startswith('#'):continue
  dest=p.parent/target.split('#',1)[0]
  if not dest.exists():errors.append([str(p.relative_to(ROOT)),'broken relative link',target])
  counts['markdown_links']+=1
result={'status':'PASS_PUBLIC_SNAPSHOT_ONLY'if not errors else'FAIL','counts':counts,'errors':errors,'runtime_or_navigation_PASS_not_implied':True}
print(json.dumps(result,ensure_ascii=False,indent=2));sys.exit(bool(errors))
