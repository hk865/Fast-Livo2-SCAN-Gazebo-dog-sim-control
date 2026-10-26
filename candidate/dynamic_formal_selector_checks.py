"""Pure strict-formal selection checks; no runtime, files, ROS or Actor."""
import copy
import unittest
from diagnostic_observation import strict_formal_enabled
from r85_dynamic_obstacle_runtime import COORDINATE_SELECTION, SEGMENT_SELECTION


def profile(version):
    formal=dict(schema=f'r85_strict_formal_f1_f3/v{version}',enabled=True,
        kind='f1_to_f3',business_rejections_must_stop=True)
    if version==1:
        formal['original_single_scope_dynamic_required']=True
        dynamic=dict(completed_same_geometry_preservation=False)
    elif version==2:
        formal['dynamic_event_lifetime']='single_completed_event_window/v1'
        dynamic=dict(schema='single_completed_event_window/v1',completed_event_window=True,
            whole_task_single_scope=False,completed_same_geometry_preservation=False)
    else:
        dynamic=copy.deepcopy(SEGMENT_SELECTION if version==3 else COORDINATE_SELECTION)
        formal['dynamic_event_lifetime']=dynamic['schema']
    return dict(r85_formal_regression=formal,r85_dynamic_event_contract=dynamic,
        diagnostic_observe_only=dict(enabled=False,formal_acceptance_pass_allowed=False,
            schema='r85_existing_diagnostic_business_observation/v1'),
        r85_completed_history_async_qualification=dict(enabled=False),
        r85_session=dict(enabled=True,schema='r85_pct_scan_student_session/v1',simulation_only=True),
        r85_task=dict(kind='f1_to_f3'),r85_auto_reference_contract=dict(enabled=True,
            schema='r85_persistent_auto_reference/v1',require_child_odometry_consumption_ack=True,
            require_reference_protocol_ack=True,terminal_heading_from_pct_approach=True),
        cascade=dict(r85_student_lower_loop_ack=dict(enabled=True)),
        continuous_route_contract=dict(enabled=False),actor_kind='student',controller_kind='student',
        navigation_ground_truth_used=False,mission46_required=False)


class FormalSelectionChecks(unittest.TestCase):
    def test_all_four_exact_contracts(self):
        for version in range(1,5):
            with self.subTest(version=version):self.assertTrue(strict_formal_enabled(profile(version)))

    def test_other_schema_dynamic_contracts_do_not_cross_select(self):
        for version in range(1,5):
            for other in range(1,5):
                if version==other:continue
                p=profile(version);p['r85_dynamic_event_contract']=profile(other)['r85_dynamic_event_contract']
                with self.subTest(version=version,other=other),self.assertRaises(ValueError):strict_formal_enabled(p)

    def test_v4_formal_and_dynamic_keys_are_exact_and_typed(self):
        for field in ('r85_formal_regression','r85_dynamic_event_contract'):
            for key,value in profile(4)[field].items():
                for change in ('missing','wrong','wrong_type'):
                    p=profile(4)
                    if change=='missing':p[field].pop(key)
                    elif change=='wrong':p[field][key]=not value if type(value)is bool else value+1 if type(value)is int else value+'_wrong'
                    else:p[field][key]=int(value) if type(value)is bool else str(value) if type(value)is int else None
                    with self.subTest(field=field,key=key,change=change),self.assertRaises(ValueError):strict_formal_enabled(p)
            p=profile(4);p[field]['unexpected']=True
            with self.subTest(field=field,change='extra'),self.assertRaises(ValueError):strict_formal_enabled(p)

    def test_v4_cannot_enable_diagnostic_or_disable_existing_source_ACKs(self):
        for field,key,value in [('diagnostic_observe_only','enabled',True),
                ('r85_completed_history_async_qualification','enabled',True),
                ('r85_auto_reference_contract','require_child_odometry_consumption_ack',False),
                ('r85_auto_reference_contract','require_reference_protocol_ack',False),
                ('cascade','r85_student_lower_loop_ack',dict(enabled=False))]:
            p=profile(4);p[field][key]=value
            with self.subTest(field=field,key=key),self.assertRaises(ValueError):strict_formal_enabled(p)

    def test_legacy_disabled_and_absent_selection(self):
        self.assertFalse(strict_formal_enabled({}))
        self.assertFalse(strict_formal_enabled(dict(r85_formal_regression=dict(enabled=False))))


if __name__=='__main__':unittest.main()
