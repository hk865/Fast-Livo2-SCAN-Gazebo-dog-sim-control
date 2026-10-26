"""Single checked-out package layout; no legacy code/worktree fallback."""
from pathlib import Path
REPOSITORY = Path(__file__).resolve().parents[1]
RELEASE = REPOSITORY / 'releases/formal22'
DEPENDENCIES = RELEASE / 'dependencies'
TEACHER_ROOT = DEPENDENCIES / 'workspace/multifloor_demo/teacher_mode'
DEMO_ROOT = TEACHER_ROOT.parent
GUARD_ROOT = REPOSITORY / '.local_dependencies/storage_guard'
# Shared owned output pool only; never an executed source dependency.
RUN_STORAGE_ROOT = REPOSITORY / '.local/runs'
# Geometry source locations are explicit package assets, never a legacy checkout.
GEOMETRY_STAGE = DEPENDENCIES / 'history/2026-10-14/task/r83_demo/implementation_attempt_20261014/pct_route_stage'
GEOMETRY_HELPER = GEOMETRY_STAGE / 'validate_route_support.py'
GEOMETRY_CONFIG = GEOMETRY_STAGE / 'support_validation_config.json'
GEOMETRY_METADATA = DEPENDENCIES / 'derived_inputs/typed_route/metadata_actual_world.json'
