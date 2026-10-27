"""One project-local SCAN installation; no historical adapter/build contracts."""
from project_config import CONFIG,path
R85_SELECTOR='mode3'
def scan_contract(profile=None):
    workspace=path('scan_workspace')
    executable=workspace/'install/scan_planner/lib/scan_planner/scan_planner_node'
    config=workspace/'install/scan_planner/share/scan_planner/config/planner.yaml'
    for p in [workspace/'install/local_setup.bash',executable,config]:
        if not p.is_file():raise FileNotFoundError('SCAN dependency missing: '+str(p))
    return workspace,CONFIG,dict(workspace=str(workspace),executable=str(executable),config=str(config),protocol_version=3,actual_loaded_binary_witness_required=True)
def evidence_files(profile=None):
    workspace,_,data=scan_contract(profile)
    return [workspace/'install/local_setup.bash',__import__('pathlib').Path(data['executable']),__import__('pathlib').Path(data['config'])]
