# Publication 2026-10-13: local paths redacted; numeric/control behavior retained; local admission is not transferred.
"""Explicit R80 current-retained guard proxy; no historical fallback."""
from pathlib import Path
import hashlib,importlib.util,sys
S=Path('./validation/r83_harness')
PATH=S/'storage_guard.py'
if hashlib.sha256(PATH.read_bytes()).hexdigest()!='ebf29f18def24fe32955c93547ebd29d8cc989ded71acdb79bf2e040f91a5717':
    raise RuntimeError('R80 approved current-retained guard source changed')
sys.path.insert(0,str(S))
spec=importlib.util.spec_from_file_location('R80_current_retained_guard',PATH)
base=importlib.util.module_from_spec(spec);sys.modules[spec.name]=base;spec.loader.exec_module(base)
register,usage,capacity,observe=base.register,base.usage,base.capacity,base.observe
StorageStop,Watchdog=base.StorageStop,base.Watchdog
