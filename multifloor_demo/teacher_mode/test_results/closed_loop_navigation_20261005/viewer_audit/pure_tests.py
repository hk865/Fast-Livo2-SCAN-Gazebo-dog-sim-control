#!/usr/bin/env python3
"""Read-only dashboard receipt selection checks; no ROS or simulation."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('teacher_read_only_viewer', ROOT / 'scripts/serve.py')
VIEW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VIEW)
CHECKS = {}


def check(name, condition):
    assert condition, name
    CHECKS[name] = True


def write(directory, filename, receipt):
    (directory / filename).write_text(json.dumps(receipt), encoding='utf-8')


def receipt(directory, schema, passed=True):
    return {'schema': schema, 'run': str(directory.resolve()),
            'status': 'passed' if passed else 'failed',
            'scope': {'navigation_ground_truth_used': False},
            'checks': {'original_evidence': {'passed': passed,
                'status': 'passed' if passed else 'failed', 'native_xy_drift_m': None}}}


def main():
    common_schema = 'independent_actual_SLAM_SCAN_cascade_navigation/v1'
    ramp_schema = 'independent_actual_SLAM_SCAN_complete_ramp_navigation/v1'
    common_filename = 'summary_closed_loop_cascade_independent.json'
    ramp_filename = 'summary_closed_loop_ramp_independent.json'
    html_path = ROOT / 'web/index.html'
    html_hash = hashlib.sha256(html_path.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='teacher_viewer_read_only_') as temporary:
        base = Path(temporary)
        directory = base / 'new_actual_run'; directory.mkdir()
        check('empty_run_does_not_claim_closed_loop', VIEW.closed_loop_receipt_view(directory) == {})
        write(directory, 'navigation_scope.json', {'schema': 'teacher_closed_loop_navigation_scope/v1'})
        write(directory, 'state.json', {'status': 'succeeded'})
        write(directory, 'summary.json', {'status': 'passed', 'levels': {'navigation': 'passed'}})
        result = VIEW.closed_loop_receipt_view(directory)
        check('runtime_or_generic_pass_cannot_certify_navigation', result['status'] == 'unverified')
        write(directory, common_filename, receipt(directory, common_schema))
        write(directory, 'navigation_status.json', {'status': 'failed', 'reason': 'post-exit shutdown'})
        result = VIEW.closed_loop_receipt_view(directory)
        check('shutdown_fail_does_not_override_independent_pass', result['status'] == 'passed' and result['counts']['passed'] == 1)
        write(directory, ramp_filename, receipt(directory, ramp_schema, False))
        result = VIEW.closed_loop_receipt_view(directory)
        check('formal_ramp_failure_has_priority_over_common_pass', result['status'] == 'failed' and result['selection_kind'] == 'ramp_independent')
        wrong_run = receipt(directory, ramp_schema); wrong_run['run'] = str(base / 'old_truth_run')
        write(directory, ramp_filename, wrong_run)
        result = VIEW.closed_loop_receipt_view(directory)
        check('cross_run_receipt_rejected_without_pass_fallback', result['status'] == 'unverified' and result['counts']['passed'] == 0)
        malformed = receipt(directory, ramp_schema); malformed['checks']['original_evidence']['passed'] = None
        write(directory, ramp_filename, malformed)
        check('claimed_pass_with_unknown_check_is_rejected', VIEW.closed_loop_receipt_view(directory)['status'] == 'unverified')
        malformed = receipt(directory, ramp_schema); malformed['scope'] = {}
        write(directory, ramp_filename, malformed)
        check('missing_actual_navigation_source_declaration_rejected', VIEW.closed_loop_receipt_view(directory)['status'] == 'unverified')
        malformed = receipt(directory, ramp_schema); malformed['schema'] = common_schema
        write(directory, ramp_filename, malformed)
        check('receipt_type_schema_mismatch_rejected', VIEW.closed_loop_receipt_view(directory)['status'] == 'unverified')
        (directory / ramp_filename).unlink(); (directory / common_filename).unlink()
        write(directory, 'summary_closed_loop_ramp_independent.pilot_v1.json', receipt(directory, ramp_schema, False))
        result = VIEW.closed_loop_receipt_view(directory)
        check('pilot_fail_retained_but_cannot_certify_missing_formal_receipt', result['status'] == 'unverified' and result['pilot_diagnostics'][0]['status'] == 'failed')
        write(directory, 'summary_closed_loop_ramp_independent.pilot_v1.json', receipt(directory, ramp_schema))
        check('pilot_pass_never_upgrades_navigation', VIEW.closed_loop_receipt_view(directory)['status'] == 'unverified')
        old = base / 'old_truth_run'; old.mkdir()
        write(old, 'summary.json', {'status': 'passed'})
        write(old, 'summary_truth_pid.json', {'status': 'passed'})
        check('old_truth_and_generic_receipts_not_classified_actual_closed_loop', VIEW.closed_loop_receipt_view(old) == {})
        r = receipt(directory, common_schema)
        r['checks']['unknown'] = {'status': 'unverified', 'passed': None}
        r['checks']['failure'] = {'status': 'failed', 'passed': False}
        r['status'] = 'failed'; write(directory, common_filename, r)
        result = VIEW.closed_loop_receipt_view(directory)
        check('pass_fail_unknown_counts_preserved', result['counts'] == {'passed': 1, 'failed': 1, 'unverified': 1, 'total': 3})
        snap = VIEW.Dashboard(base).snapshot(directory.name)
        check('snapshot_contains_original_and_selected_receipts', snap['closed_loop_result']['common']['raw'] == r and snap['closed_loop_result']['status'] == 'failed')
        check('run_listing_uses_independent_status', next(x for x in VIEW.Dashboard(base).directories() if x['id'] == directory.name)['status'] == 'failed')
        for suffix, expected, counts in [
            ('20261005_124942_closed_loop_cascade_actual_flat_r1_54a1', 'passed', (18, 0, 0)),
            ('20261005_125207_closed_loop_cascade_actual_flat_r2_5011', 'passed', (18, 0, 0)),
            ('20261005_125454_closed_loop_cascade_actual_flat_r3_a9a2', 'failed', (17, 1, 0)),
            ('20261005_125808_closed_loop_cascade_actual_ramp12_up_r1_b318', 'failed', (14, 3, 2)),
        ]:
            result = VIEW.closed_loop_receipt_view(ROOT / 'runs' / suffix)
            check('actual_' + suffix, result['status'] == expected and
                  tuple(result['counts'][key] for key in ('passed', 'failed', 'unverified')) == counts)
        result = VIEW.closed_loop_receipt_view(ROOT / 'runs' / '20261005_125808_closed_loop_cascade_actual_ramp12_up_r1_b318')
        check('actual_ramp_pilot_is_separately_retained_failure', len(result['pilot_diagnostics']) == 1 and result['pilot_diagnostics'][0]['status'] == 'failed')
        page = VIEW.dashboard_page().decode()
        scripts = re.findall(r'<script>([\s\S]*?)</script>', page)
        script_file = base / 'dashboard_syntax.js'; script_file.write_text('\n'.join(scripts))
        subprocess.run(['node', '--check', str(script_file)], check=True, capture_output=True)
        check('combined_historical_and_additive_javascript_syntax', True)
        harness = r'''
const assert=require('node:assert/strict');
class Element{constructor(){this.textContent='';this.children=[];this.hidden=false;}append(...x){this.children.push(...x);}after(x){elements[x.id]=x;}replaceChildren(...x){this.children=x;}}
const elements={};const $=id=>elements[id]||(elements[id]=new Element());
const document={createElement:()=>new Element()};
let current=null;let legacyCalls=0;let shown=null;
function show(data){current=data;legacyCalls++;$('raw').textContent='{}';navigationPlot();}
function showTests(x){shown=x;}
function checkName(x){return x;}
function checkDetail(x){return x.reason;}
function status(value){let x=typeof value==='object'?value?.status:value;return x==='passed'?['通过','pass']:x==='failed'?['失败','fail']:['未验证',''];}
function navigationPlot(){$('navigationCaption').textContent='old unknown or shutdown failure';}
'''
        patch = VIEW.CLOSED_LOOP_PAGE_SCRIPT.removeprefix('\n<script>').removesuffix('</script>\n')
        harness += '\n' + patch + r'''
const receipt={filename:'summary_closed_loop_cascade_independent.json',sha256:'abc',status:'passed',counts:{total:18,passed:18,failed:0,unverified:0},raw:{status:'passed'}};
const data={closed_loop_result:{status:'passed',selection_kind:'cascade_independent',selected_validation_summary:{checks:{all:{status:'passed'}}},display_note:'independent',selected_receipt_filename:receipt.filename,selected_receipt_sha256:'abc',common:receipt},navigation:{state:{status:'failed'},poses:{records:323},scan:{hash_verified:true,trajectory_id:10}}};
show(data);assert.equal(legacyCalls,1);assert.equal(shown,data.closed_loop_result.selected_validation_summary);assert.match($('navigationCaption').textContent,/独立验收：通过/);assert.match($('navigationCaption').textContent,/最后运行状态：failed/);navigationPlot();assert.match($('navigationCaption').textContent,/独立验收：通过/);assert.equal($('closedLoopReceipts').hidden,false);show({});assert.equal(legacyCalls,2);assert.equal($('closedLoopReceipts').hidden,true);
console.log('additive DOM receipt renderer PASS');
'''
        script_file.write_text(harness)
        subprocess.run(['node', str(script_file)], check=True, capture_output=True)
        check('additive_dom_formal_receipt_shutdown_resize_and_old_run', True)
    check('historical_html_source_unchanged', hashlib.sha256(html_path.read_bytes()).hexdigest() == html_hash)
    output = {'schema': 'teacher_closed_loop_viewer_pure_checks/v1', 'status': 'passed',
              'checks': CHECKS, 'count': len(CHECKS),
              'serve_sha256': hashlib.sha256((ROOT / 'scripts/serve.py').read_bytes()).hexdigest(),
              'historical_html_sha256': html_hash, 'no_ROS_no_Gazebo_no_service_restart': True}
    destination = Path(__file__).with_name('pure_test_result.json')
    destination.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(output, ensure_ascii=False))


if __name__ == '__main__':
    main()
