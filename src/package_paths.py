"""Canonical project paths; historical directories never enter runtime lookup."""
from pathlib import Path
REPOSITORY=Path(__file__).resolve().parents[1]
TEACHER_ROOT=REPOSITORY
DEMO_ROOT=REPOSITORY
NAVIGATION_ROOT=REPOSITORY/'src'
RUN_STORAGE_ROOT=REPOSITORY/'runs'
GEOMETRY_STAGE=REPOSITORY/'assets/geometry'
GEOMETRY_HELPER=NAVIGATION_ROOT/'validate_route_support.py'
GEOMETRY_CONFIG=GEOMETRY_STAGE/'limits.json'
GEOMETRY_METADATA=GEOMETRY_STAGE/'metadata.json'
