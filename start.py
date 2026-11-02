#!/usr/bin/env python3
"""Single entry for the clean Building navigation project."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'src'))
from bringup.building import main
if __name__=='__main__':raise SystemExit(main())
