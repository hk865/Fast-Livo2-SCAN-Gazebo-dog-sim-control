#!/usr/bin/env python3
"""Use the unchanged production timing observer around the staged health bridge."""
import os
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
from control_timing_trace import Trace,install
import bridge_with_feedback as module

def main():
    if os.environ.get('DEMO_CONTROL_TIMING_AUDIT')!='1':
        raise RuntimeError('Both A and B require identical optional timing capture')
    trace=Trace(Path(os.environ['DEMO_RUN_DIR']),'bridge')
    # install inspects module.Bridge; the actual instantiated class is the
    # staged subclass. Its super() remains the original production class.
    module.Bridge=module.FeedbackBridge
    install(module,'bridge',trace)
    module.FeedbackBridge.on_feedback=trace.wrap('callback.on_feedback',module.FeedbackBridge.on_feedback)
    try:module.main()
    finally:trace.save()

if __name__=='__main__':main()
