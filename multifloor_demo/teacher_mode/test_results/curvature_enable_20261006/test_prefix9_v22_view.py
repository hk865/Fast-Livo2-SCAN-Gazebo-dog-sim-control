"""Inherited scope/hash gates plus exact V21 helper routing; synthetic only."""
import importlib.util
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / 'corridor_tracking_v20_continue_20261006/viewer/test_prefix9_view.py'
spec = importlib.util.spec_from_file_location('previous_prefix_panel_tests', OLD)
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)


class V22PanelTests(old.Prefix9PanelTests):
    def version(self, selector):
        old.write(self.run / 'navigation_profile.json', {
            'original46_prefix_regions': 9, 'controller_selector': selector})
        for binding in self.raw['source_bindings']:
            if binding['file'] == str(self.run / 'navigation_profile.json'):
                binding['sha256'] = old.digest(self.run / 'navigation_profile.json')
            elif selector == 'v22_curvature_archive' and 'navigation/corridor_tracking_v20/' in binding['file']:
                prior = Path(binding['file'])
                new = self.root / 'navigation/corridor_tracking_v22_curvature_archive' / prior.name
                new.parent.mkdir(parents=True, exist_ok=True); new.write_bytes(prior.read_bytes())
                binding.update(file=str(new), sha256=old.digest(new))

    def test_v22_selected_helper_roots_are_accepted(self):
        self.version('v22_curvature_archive')
        self.save()
        self.assertTrue(self.view()['limited_prefix9_pass'])
        self.assertFalse(self.view()['full46_pass'])

    def test_v22_cannot_take_v20_live_helpers(self):
        self.version('v22_curvature_archive')
        path = self.root / 'navigation/corridor_tracking_v20/route.py'
        path.parent.mkdir(parents=True, exist_ok=True); path.write_text('# wrong version\n')
        self.raw['source_bindings'].append({'file': str(path), 'sha256': old.digest(path)})
        self.save(); self.assertFalse(self.view()['valid_for_selected_run'])

    def test_unknown_selector_has_no_scoped_result(self):
        self.version('unreviewed_controller'); self.save()
        self.assertEqual(self.view(), {})

    def test_all_helper_bindings_are_mandatory(self):
        self.version('v22_curvature_archive')
        self.raw['source_bindings'] = [b for b in self.raw['source_bindings'] if Path(b['file']).is_relative_to(self.run)]
        self.save(); self.assertFalse(self.view()['valid_for_selected_run'])

    def test_each_helper_binding_is_mandatory(self):
        self.version('v22_curvature_archive')
        original = list(self.raw['source_bindings'])
        for helper in [b for b in original if not Path(b['file']).is_relative_to(self.run)]:
            self.raw['source_bindings'] = [b for b in original if b is not helper]
            self.save(); self.assertFalse(self.view()['valid_for_selected_run'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
