"""Single checked-out package layout; no legacy code/worktree fallback."""
from pathlib import Path
REPOSITORY = Path(__file__).resolve().parents[1]
RELEASE = REPOSITORY / 'releases/formal22'
DEPENDENCIES = RELEASE / 'dependencies'
TEACHER_ROOT = DEPENDENCIES / 'workspace/multifloor_demo/teacher_mode'
DEMO_ROOT = TEACHER_ROOT.parent
GUARD_ROOT = REPOSITORY / '.local_dependencies/storage_guard'
# Shared owned output pool only; never an executed source dependency.
RUN_STORAGE_ROOT = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs')
