"""Direct immutable geometry dependencies for the bounded V22 experiment.

This verifies source evidence only. It does not authorize navigation success.
"""
from pathlib import Path
import hashlib


def geometry_helper_files(here):
    here = Path(here).resolve()
    if len(here.parents) < 3:
        raise RuntimeError('Invalid mandatory candidate geometry root')
    demo = here.parents[2]
    return [here/'route.py', here/'prefix_contract.py', here/'mission46_profile.py',
            demo/'navigation/goal_regions.py', demo/'mission/route_regions.py']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_geometry_archive(run, references, snapshots, manifest, here):
    """Require all five direct sources, canonical copies, and both manifest keys."""
    run = Path(run).resolve()
    root = Path(here).resolve().parents[1]
    verified = {}
    for original in geometry_helper_files(here):
        original = original.resolve()
        source = str(original)
        expected = references.get(source)
        row = snapshots.get(source)
        if (not isinstance(expected, str) or len(expected) != 64
                or not isinstance(row, dict) or row.get('sha256') != expected
                or not isinstance(row.get('snapshot'), str)):
            raise RuntimeError('Missing direct geometry source or snapshot: ' + source)
        rel = (original.relative_to(root) if original.is_relative_to(root)
               else Path('external')/(expected[:16]+'_'+original.name))
        canonical = run/'sources'/rel
        target = Path(row['snapshot']).resolve()
        if (target != canonical.resolve() or not target.is_relative_to(run/'sources')
                or canonical.is_symlink() or not canonical.is_file()
                or references.get(str(target)) != expected
                or manifest.get(str(rel)) != expected or manifest.get(source) != expected
                or sha(original) != expected or sha(target) != expected):
            raise RuntimeError('Incomplete direct geometry archive: ' + source)
        verified[source] = {'snapshot': str(target), 'sha256': expected}
    return verified
