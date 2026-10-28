#!/usr/bin/env python3
"""Start the current project; developer validation lives under tools/validation."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src'))
from bringup.main import main
if __name__=='__main__':raise SystemExit(main())
