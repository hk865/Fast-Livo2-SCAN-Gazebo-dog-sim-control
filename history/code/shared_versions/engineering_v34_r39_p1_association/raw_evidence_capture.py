#!/usr/bin/env python3
"""Independent recording-only helper; ROS subscriptions, no publishers."""
from pathlib import Path
import sys
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from evidence_runtime_v32.raw_capture import main
if __name__=='__main__':main()
