"""One editable configuration and project-relative asset locations."""
from pathlib import Path
import json,os
ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/navigation.json'
def expand(value):
    if isinstance(value,dict):return {k:expand(v)for k,v in value.items()}
    if isinstance(value,list):return [expand(v)for v in value]
    if isinstance(value,str):return value.replace('${PROJECT_ROOT}',str(ROOT))
    return value
def loads(raw,*args,**kwargs):return expand(json.loads(raw,*args,**kwargs))
def read():
    value=loads(CONFIG.read_text())
    if not isinstance(value,dict):raise ValueError('Navigation configuration must be an object')
    return value
def path(name):
    setting=read()['runtime_paths'][name]
    if name=='actor_python'and setting=='auto':
        roots=[Path(os.environ.get('ISAAC_SIM_PATH','/opt/isaacsim')),Path.home()/'isaacsim']
        for root in roots:
            value=root/'kit/python/bin/python3'
            if value.is_file():return value.resolve()
        raise FileNotFoundError('Set ISAAC_SIM_PATH to the existing public simulator/PyTorch installation')
    value=Path(setting).expanduser()
    return value.resolve()if value.is_absolute()else(ROOT/value).resolve()
