#!/usr/bin/env python3
"""Independent bounded V22 archive checks; only source files and temporary fixtures."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile

HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
V21 = TEACHER / 'navigation/corridor_tracking_v21_curvature'
V22 = TEACHER / 'navigation/corridor_tracking_v22_curvature_archive'
EVALUATOR = TEACHER / 'test_results/corridor_tracking_v20_20261006/evaluation/evaluate_prefix9.py'
EVALUATOR_SHA = '812946d475141e2bd2c71fb925cadb75b6b1086027167fb53d8a318b25c3481b'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load(p, name):
    spec = importlib.util.spec_from_file_location(name, p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_checks():
    geometry = load(V22 / 'geometry_archive.py', 'independent_v22_geometry')
    checks = []
    def check(name, predicate):
        if not predicate:
            raise AssertionError(name)
        checks.append(name)
    originals = geometry.geometry_helper_files(V22)
    check('exact_five_helper_paths', originals == [V22 / n for n in
        ('route.py', 'prefix_contract.py', 'mission46_profile.py')] + [
        TEACHER.parent / 'navigation/goal_regions.py', TEACHER.parent / 'mission/route_regions.py'])
    profile_name = 'profiles/curvature_original46_prefix9_on.json'
    old = json.loads((V21 / profile_name).read_text())
    new = json.loads((V22 / profile_name).read_text())
    normalized = copy.deepcopy(new)
    for key in ('controller_selector', 'profile_version', 'prospective_change'):
        normalized[key] = old[key]
    normalized['mission46_required_source_files'] = [
        x.replace(str(V22), str(V21)) for x in normalized['mission46_required_source_files']]
    check('profile_all_control_geometry_and_other_fields_exact', normalized == old)
    check('selector_is_new_candidate', new['controller_selector'] == 'v22_curvature_archive')
    mutable = {'pid_scope.py', 'corridor_preflight.py', 'run.py', 'wiring_tests.py'}
    identical = []
    for path in sorted(V21.glob('*.py')):
        if path.name not in mutable:
            check('byte_identical_' + path.name, (V22 / path.name).read_bytes() == path.read_bytes())
            identical.append(path.name)
    def without_doc(p):
        tree = ast.parse(p.read_text())
        if isinstance(tree.body[0], ast.Expr) and isinstance(tree.body[0].value, ast.Constant):
            tree.body = tree.body[1:]
        return ast.dump(tree, include_attributes=False)
    check('runner_executable_AST_unchanged', without_doc(V22 / 'run.py') == without_doc(V21 / 'run.py'))
    check('frozen_eight_gate_evaluator_hash_unchanged', sha(EVALUATOR) == EVALUATOR_SHA)
    evaluator = load(EVALUATOR, 'independent_v22_frozen_evaluator')
    # The evaluator still imports identical V20 geometry; CORE only controls
    # the exact live helper bindings, as in the reviewed version adapter.
    for name in ('route.py', 'prefix_contract.py', 'mission46_profile.py'):
        check('original_evaluator_geometry_identical_' + name,
              (evaluator.CORE / name).read_bytes() == (V22 / name).read_bytes())
    evaluator.CORE = V22
    with tempfile.TemporaryDirectory(prefix='v22_archive_finite_') as tmp:
        base = Path(tmp)
        here = base / 'demo/teacher/navigation/candidate'
        run = base / 'run'
        fake_helpers = geometry.geometry_helper_files(here)
        refs, snapshots, manifest = {}, {}, {}
        for original, fake in zip(originals, fake_helpers):
            fake.parent.mkdir(parents=True, exist_ok=True)
            fake.write_bytes(original.read_bytes())
            h = sha(fake)
            root = here.parents[1]
            rel = fake.relative_to(root) if fake.is_relative_to(root) else Path('external') / (h[:16] + '_' + fake.name)
            target = run / 'sources' / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(fake.read_bytes())
            refs[str(fake)] = refs[str(target)] = h
            snapshots[str(fake)] = {'snapshot': str(target), 'sha256': h}
            manifest[str(fake)] = manifest[str(rel)] = h
        check('all_five_canonical_archive_positive', len(geometry.verify_geometry_archive(run, refs, snapshots, manifest, here)) == 5)
        for helper in fake_helpers:
            source = str(helper)
            target = Path(snapshots[source]['snapshot'])
            rel = str(target.relative_to(run / 'sources'))
            for dimension in ('source_ref', 'snapshot', 'snapshot_ref', 'relative_manifest', 'absolute_manifest'):
                r, s, m = copy.deepcopy(refs), copy.deepcopy(snapshots), copy.deepcopy(manifest)
                {'source_ref': r, 'snapshot': s, 'snapshot_ref': r,
                 'relative_manifest': m, 'absolute_manifest': m}[dimension].pop(
                    {'source_ref': source, 'snapshot': source, 'snapshot_ref': str(target),
                     'relative_manifest': rel, 'absolute_manifest': source}[dimension])
                try:
                    geometry.verify_geometry_archive(run, r, s, m, here)
                except RuntimeError:
                    checks.append('reject_missing_' + dimension + '_' + helper.name)
                else:
                    raise AssertionError('missing dependency accepted: ' + dimension)
            for p, kind in ((helper, 'live_bytes'), (target, 'snapshot_bytes')):
                saved = p.read_bytes()
                try:
                    p.write_bytes(saved + b'\n# synthetic mutation\n')
                    try:
                        geometry.verify_geometry_archive(run, refs, snapshots, manifest, here)
                    except RuntimeError:
                        checks.append('reject_changed_' + kind + '_' + helper.name)
                    else:
                        raise AssertionError('changed dependency accepted')
                finally:
                    p.write_bytes(saved)
            alternate = run / 'sources' / ('wrong_path_' + helper.name)
            alternate.write_bytes(target.read_bytes())
            s = copy.deepcopy(snapshots)
            s[source]['snapshot'] = str(alternate)
            try:
                geometry.verify_geometry_archive(run, refs, s, manifest, here)
            except RuntimeError:
                checks.append('reject_same_hash_wrong_snapshot_path_' + helper.name)
            else:
                raise AssertionError('noncanonical same-byte snapshot accepted')
        # Exercise the unchanged independent acceptance against real helper
        # bytes using its actual Sources binder, without any actual run logs.
        archive = base / 'evaluation_fixture'
        (archive / 'sources').mkdir(parents=True)
        manifest2 = {}
        targets = []
        for i, original in enumerate(originals):
            key = str(i) + '_' + original.name
            target = archive / 'sources' / key
            target.write_bytes(original.read_bytes())
            manifest2[key] = manifest2[str(original)] = sha(target)
            targets.append(target)
        check('frozen_verify_archive_all_five_positive', evaluator.verify_archive(archive, evaluator.Sources(), manifest2)['passed'])
        for target in targets:
            raw = target.read_bytes()
            target.unlink()
            check('frozen_verify_archive_reject_missing_' + target.name,
                  not evaluator.verify_archive(archive, evaluator.Sources(), manifest2)['passed'])
            target.write_bytes(raw)
        orphan = dict(manifest2)
        orphan['/synthetic/orphan_alias.py'] = '0' * 64
        check('frozen_verify_archive_reject_orphan_alias', not evaluator.verify_archive(archive, evaluator.Sources(), orphan)['passed'])
    return dict(schema='curvature_v22_independent_archive_finite/v1', passed=True,
        checks=checks, check_count=len(checks), unchanged_python_sources=identical,
        source_bindings={str(p):sha(p) for p in [Path(__file__), V22/'geometry_archive.py',
            V22/'pid_scope.py', V22/'corridor_preflight.py', V22/'run.py', V21/profile_name,
            V22/profile_name, EVALUATOR, *originals]},
        actual_navigation_verified=False, actual_raw_read=False, ROS_or_Gazebo_started=False)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run_checks()
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'passed': result['passed'], 'checks': result['check_count'], 'output': str(args.output)}))
