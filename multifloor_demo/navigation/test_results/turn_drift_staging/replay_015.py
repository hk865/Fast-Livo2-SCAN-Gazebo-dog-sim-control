import json,pathlib
import replay
from turn_drift import TurnDriftSupervisor
class Profile(TurnDriftSupervisor):
    LIMITS=dict(TurnDriftSupervisor.LIMITS,planar_drift_m=.15,persistence_ns=200_000_000,min_raw_observations=3)
    def __init__(self,require_path_offset=False):super().__init__(require_path_offset=False,profile="displacement_015_02")
replay.TurnDriftSupervisor=Profile
ROOT=replay.ROOT
result=[]
for name,run,component in [("a_disabled_01502",ROOT/"simulation/test_results/20261002_feedback_first4_a_disabled",True),("b_enabled_01502",ROOT/"simulation/test_results/20261002_feedback_first4_b_enabled",True),("full18_01502",ROOT/"runs/20261002_001559_882ddb",False)]:result.append(replay.replay(name,run,component,False))
pathlib.Path(__file__).with_name("replay_015_summary.json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2))
