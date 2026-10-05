"""Portable clone gate: original reports are historical only."""
import sys
from pathlib import Path
REPO=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(REPO/'tools'))
from portable_common import evidence_files as _evidence,verify_local_preflight
def evidence_files(here):return _evidence(here)
def verify_preflight(here,profile):return verify_local_preflight(here,profile)
