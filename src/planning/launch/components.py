from common.config import path
from bringup.launching import python,owned
def components(run):
    prefix=path('ros_prefix')
    return owned('scan_supervisor',python('planning.scan_epoch_supervisor','--run',run,
        '--planner',prefix/'lib/scan_planner/scan_planner_node','--config',prefix/'share/scan_planner/config/planner.yaml',
        '--params',run/'v34_scan_overrides.yaml'),run)
