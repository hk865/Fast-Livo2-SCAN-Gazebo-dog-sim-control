"""Finite caps only; preserve all original task and physical failures."""
NATIVE_CAP_REASON='R77_native1500s_full_cap'
WALL_CAP_REASON='R77_wall4350s_full_cap'
OUTPUT_CAP_REASON='R77_predictive_round_stop'
NATIVE_CAP_NS=1_500_000_000_000
ACQUISITION_WALL_CAP_S=4350.
OUTPUT_SOFT_B=6_800_000_000
OUTPUT_HARD_B=10_000_000_000
from capture_owner_stop import OWNER_STOP_REASONS
