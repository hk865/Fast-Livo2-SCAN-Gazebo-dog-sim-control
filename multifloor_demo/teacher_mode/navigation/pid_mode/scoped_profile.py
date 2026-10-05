#!/usr/bin/env python3
"""Public CLI/API for independent measured-SLAM PID experiments."""
try:
    from .pid_scope import *
except ImportError:
    from pid_scope import *
if __name__=="__main__":main()
