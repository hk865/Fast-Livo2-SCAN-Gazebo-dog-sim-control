"""Record actual module paths and selected features, without source version gates."""
import json,os,sys,time
from pathlib import Path
def write_binding(node):
    modules={}
    here=Path(__file__).resolve().parent
    for name,module in list(sys.modules.items()):
        source=getattr(module,'__file__',None)
        if source and Path(source).resolve().is_relative_to(here):modules[name]=dict(path=str(Path(source).resolve()))
    value=dict(run_id=Path(node.run).name,modules=modules,actual_controller_class=type(node).__name__,before_executor_spin=True,control_or_navigation_PASS=False)
    (Path(node.run)/'r86_runtime_binding.json').write_text(json.dumps(value,indent=2)+'\n')
def finish_empty_retry_yield(node,executor):
    """Exit-only receipt after owner stop and pool join; preserve existing outcome."""
    try:
        candidate=getattr(node,'_r91_empty_retry_yield',None)
        if candidate is None:
            return
        statistics=candidate.snapshot()
        spinner_native_tid=executor.r90_statistics()['spinner_native_tid']
        value=dict(schema='r91_empty_retry_yield_statistics/v1',
            selection=dict(node._r91_executor_receipt),statistics=statistics,
            spinner_native_tid=spinner_native_tid,
            owner_matches_spinner=statistics['owner_tid']==spinner_native_tid)
        (Path(node.run)/'r91_empty_retry_yield_statistics.json').write_text(
            json.dumps(value,allow_nan=False)+'\n')
    except Exception:
        # Missing/failed evidence is a verification failure, not a new shutdown result.
        try:node._r91_statistics_write_failed=True
        except Exception:pass
