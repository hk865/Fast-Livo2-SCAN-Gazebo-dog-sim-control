"""Import application modules without launching nodes, workers or simulation."""
from pathlib import Path
import importlib,json,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
def main():
    modules=[];errors=[]
    for p in sorted((ROOT/'src').rglob('*.py')):
        rel=p.relative_to(ROOT/'src')
        if any(part in ('frontend','scan','sdk') for part in rel.parts) or p.name=='__init__.py' or '.launch.' in p.name:continue
        name='.'.join(rel.with_suffix('').parts);modules.append(name)
        try:importlib.import_module(name)
        except Exception as e:errors.append(dict(module=name,error=type(e).__name__+': '+str(e)))
    loaded=[name for name in sys.modules if name=='experiments' or name.startswith('experiments.')]
    result=dict(checked=len(modules),errors=errors,experiment_modules_loaded=loaded,passed=not errors and not loaded,
        nodes_started=False,simulation_started=False)
    print(json.dumps(result,indent=2,ensure_ascii=False));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
