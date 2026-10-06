# Current selected-profile entrypoint; original .25/.4 tests are archived as
# test_turn_drift_025_v1.py with their original result.
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).with_name("test_candidate.py")),run_name="__main__")
