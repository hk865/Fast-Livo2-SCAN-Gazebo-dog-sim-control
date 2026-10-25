#!/usr/bin/env python3
"""Read-only, standard-library check of the public formal22 source snapshot.
No module imports from the snapshot; no ROS, model, build or simulator startup.
"""
from pathlib import Path
import ast,hashlib,json,re,sys
ROOT=Path(__file__).resolve().parents[1]
RELEASE=ROOT/'releases/formal22'
manifest=json.loads((RELEASE/'SOURCE_EXPORT.json').read_text())
errors=[];upstream_fragment_links=[];counts={'hashed_files':0,'python_syntax':0,'python_nonstring_AST':0,'json_files':0,'markdown_links':0}
class NonStringAST(ast.NodeTransformer):
 def visit_Constant(self,node):
  return ast.copy_location(ast.Constant(value='<STRING>'),node) if isinstance(node.value,str) else node
for name,row in manifest['files'].items():
 p=RELEASE/name
 if not p.is_file():errors.append([name,'missing']);continue
 if hashlib.sha256(p.read_bytes()).hexdigest()!=row['published_sha256']:errors.append([name,'SHA mismatch'])
 counts['hashed_files']+=1
 try:
  if p.suffix=='.py':
   tree=ast.parse(p.read_text(),filename=name);counts['python_syntax']+=1
   if row.get('nonstring_AST_sha256_original'):
    actual=hashlib.sha256(ast.dump(NonStringAST().visit(tree),include_attributes=False).encode()).hexdigest()
    if actual!=row['nonstring_AST_sha256_original']:errors.append([name,'nonstring AST mismatch'])
    counts['python_nonstring_AST']+=1
  elif p.suffix=='.json':json.loads(p.read_text());counts['json_files']+=1
 except (SyntaxError,ValueError)as e:errors.append([name,type(e).__name__,str(e)[:180]])
paths=[ROOT/'README.md',*sorted((ROOT/'docs').glob('*FORMAL22*.md')),ROOT/'docs/EXPERIMENT_HISTORY.md',RELEASE/'README.md']
paths+=[ROOT/'docs/PUBLISHED_DIRECTORY_REPRODUCTION.md']
paths+=list(RELEASE.rglob('*.md'))
for p in sorted(set(paths)):
 for target in re.findall(r'!?\[[^\]\n]+\]\(([^)\n]+)\)',p.read_text()):
  target=target.strip('<>')
  if re.match(r'^[A-Za-z][A-Za-z0-9+.-]*:',target)or target.startswith('#'):continue
  dest=p.parent/target.split('#',1)[0]
  if not dest.exists():
   if p.is_relative_to(RELEASE/'dependencies/exact_source_inputs'):
    upstream_fragment_links.append([str(p.relative_to(ROOT)),target])
   else:errors.append([str(p.relative_to(ROOT)),'broken relative link',target])
  counts['markdown_links']+=1
result={'status':'PASS_PUBLIC_SNAPSHOT_ONLY'if not errors else'FAIL','counts':counts,'errors':errors,'frozen_upstream_fragment_links_without_original_package_context':upstream_fragment_links,'upstream_fragment_scope':'Exact historical upstream bytes are preserved; their original repository-relative context is not a packaged project document. These unresolved links remain explicitly listed.','runtime_or_navigation_PASS_not_implied':True}
print(json.dumps(result,ensure_ascii=False,indent=2));sys.exit(bool(errors))
