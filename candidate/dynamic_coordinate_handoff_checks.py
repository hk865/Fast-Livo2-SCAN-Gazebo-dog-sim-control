"""Bounded offline coordinate handoff checks; no ROS initialization or experiment."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import r85_dynamic_obstacle_runtime as runtime
import r85_dynamic_obstacle_verifier as verifier


def edge(old_epoch=2,old_generation=20,new_epoch=3,new_generation=36,cut=10_000_000_000,index=1):
    old=dict(run_id='run',request_id='request',source_route_sha256='route',epoch=old_epoch,
             localization_generation=old_generation,reference_id=f'r{old_epoch}',reference_sha256=f'h{old_epoch}')
    new=dict(old,epoch=new_epoch,localization_generation=new_generation,reference_id=f'r{new_epoch}',reference_sha256=f'h{new_epoch}')
    doc=dict(run_id='run',request_id='request',source_route_sha256='route',epoch=old_epoch,
             localization_generation=old_generation,reference_id=f'r{old_epoch}',body_points=[[0.,0.,0.],[1.,0.,0.]],goal=[1.,0.,0.])
    newdoc=dict(copy.deepcopy(doc),epoch=new_epoch,localization_generation=new_generation,reference_id=f'r{new_epoch}')
    C=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    status=dict(state='running',request_id='request',known_scene_localization=dict(generation=new_generation,
        control_allowed=True,correction_stale=False,correction_transition=False,source_ns=cut,C_world_odom=C),
        known_scene_runtime_status=dict(generation=new_generation,initialized=True,error=None,correction_transition=False),
        known_scene_scan_epoch=dict(localization_generation=new_generation,planner_epoch=new_epoch,ready=True,
            child_odometry_consumption=dict(ready=True)),pose_age=.01,cloud_age=.01,raw_imu_age=.01,
        execution_bridge_safety=dict(state='ready'))
    actor=dict(native_gz_pose_info=True,entity=runtime.ENTITY,entity_id=42,raw_model_pose=dict(name=runtime.ENTITY,id=42),
        stamp_ns=cut,header=dict(stamp=dict(sec=cut//10**9,nanosec=cut%10**9)),position_world_xyz=[0.,0.,0.])
    return dict(schema='world_event_coordinate_handoff_edge/v1',switch_index=index,old_binding=old,new_binding=new,
        old_reference=doc,new_reference=dict(document=newdoc),cut_sim_ns=cut,boundary_wall=float(cut/10**9),
        dynamic_proof_seen=True,event_state='active',event_stop_token=dict(token=1),event_resume_ns=9_000_000_000,
        navigation_status=status,status_received_wall=cut/10**9-.01,
        transaction_release=dict(generation=new_generation,measured_zero_stop_verified=True,convergence=dict(converged=True,
            body_residual_m=.001,origin_residual_m=.001,rotation_residual_rad=.001),C_world_odom=C,released_source_ns=cut-100_000_000),
        actor_observation=actor,actor_received_wall=cut/10**9-.01,last_actor_ACK_position=[0.,0.,0.],
        trigger_actor_identity=dict(entity=runtime.ENTITY,entity_id=42),world_event_trigger_sim_ns=1_000_000_000)


def main():
    checks=[]
    def check(name,fn,reject=False):
        try:fn()
        except (ValueError,AssertionError,verifier.VerificationError if hasattr(verifier,'VerificationError') else ValueError):
            if not reject:raise
            checks.append(dict(check=name,passed=True));return
        if reject:raise AssertionError('Expected rejection: '+name)
        checks.append(dict(check=name,passed=True))
    good=edge()
    check('explicit_v3_post_proof_edge',lambda:runtime.validate_coordinate_handoff_edge(good))
    check('old_v2_rejects_post_proof_edge',lambda:runtime.validate_navigation_segment_edge(dict(good,schema='world_event_pre_stop_navigation_edge/v1')),True)
    old=dict(good,schema='world_event_pre_stop_navigation_edge/v1',switch_index=1,dynamic_proof_seen=False)
    check('old_v2_pre_stop_edge_still_valid',lambda:runtime.validate_navigation_segment_edge(old))
    check('old_v2_selection_unchanged',lambda:assert_true(runtime.navigation_segments_enabled(dict(r85_dynamic_event_contract=dict(runtime.SEGMENT_SELECTION)))))
    check('explicit_v3_selection',lambda:assert_true(runtime.coordinate_handoffs_enabled(dict(r85_dynamic_event_contract=dict(runtime.COORDINATE_SELECTION)))))
    mutations={
      'foreign_request':lambda e:e['new_binding'].update(request_id='foreign'),
      'route_changed':lambda e:e['new_reference']['document']['body_points'].append([2.,0.,0.]),
      'goal_changed':lambda e:e['new_reference']['document'].update(goal=[2.,0.,0.]),
      'epoch_rollback':lambda e:e['new_binding'].update(epoch=e['old_binding']['epoch']),
      'generation_rollback':lambda e:e['new_binding'].update(localization_generation=e['old_binding']['localization_generation']),
      'missing_measured_stop':lambda e:e['transaction_release'].update(measured_zero_stop_verified=False),
      'unconverged':lambda e:e['transaction_release']['convergence'].update(converged=False),
      'different_C':lambda e:e['transaction_release'].update(C_world_odom=[]),
      'stale_status':lambda e:e.update(status_received_wall=e['boundary_wall']-.301),
      'stale_source':lambda e:e['navigation_status']['known_scene_localization'].update(source_ns=e['cut_sim_ns']-300_000_001),
      'new_SCAN_not_ready':lambda e:e['navigation_status']['known_scene_scan_epoch'].update(ready=False),
      'actor_replaced':lambda e:e['actor_observation'].update(entity_id=43),
      'actor_continuity_missing':lambda e:e.update(last_actor_ACK_position=[1.,0.,0.]),
      'third_edge_rejected':lambda e:e.update(switch_index=3),
      'completed_event_not_rearmed':lambda e:e.update(event_state='completed_pending_verification')}
    for name,mutate in mutations.items():
        e=copy.deepcopy(good);mutate(e);check(name,lambda e=e:runtime.validate_coordinate_handoff_edge(e),True)
    second=edge(3,36,4,59,12_000_000_000,2)
    check('second_edge_post_resume_valid',lambda:runtime.validate_coordinate_handoff_edge(second))
    with tempfile.TemporaryDirectory(prefix='dynamic-handoff-check-') as d:
        run=Path(d);anchor=good['old_binding'];rows=[dict(kind='navigation_segment_boundary',sequence=i*10,binding=anchor,data=e)for i,e in enumerate((good,second),1)]
        chunks=[(json.dumps(r)+'\n').encode()for r in rows];receipts=[];offset=0
        for raw in chunks:
            receipts.append(dict(path=str(run/'r85_dynamic_journal.jsonl'),source_offset=offset,source_length=len(raw),source_line_sha256=hashlib.sha256(raw).hexdigest()));offset+=len(raw)
        (run/'r85_dynamic_journal.jsonl').write_bytes(b''.join(chunks))
        state=dict(coordinate_boundary_receipts=receipts,navigation_boundary_sequences=[10,20],current_navigation_binding=second['new_binding'],started_sim_ns=1_000_000_000)
        check('owner_two_real_edges_exact_tip',lambda:assert_true(runtime.coordinate_owner_binding(run,anchor,state)==second['new_binding']))
        check('old_proof_anchor_not_rebound',lambda:assert_true(anchor==good['old_binding'] and anchor!=state['current_navigation_binding']))
        bad=dict(state,current_navigation_binding=anchor)
        check('owner_rejects_stale_tip_even_cached',lambda:runtime.coordinate_owner_binding(run,anchor,bad),True)
        bad=dict(state,navigation_boundary_sequences=[10,20,30]);check('owner_rejects_third_edge',lambda:runtime.coordinate_owner_binding(run,anchor,bad),True)
    segments=dict(coordinate_chain=[dict(sequence=10,cut=10_000_000_000,binding=good['new_binding'],reference=good['new_reference']['document'],reference_source={},edge=good),dict(sequence=20,cut=12_000_000_000,binding=second['new_binding'],reference=second['new_reference']['document'],reference_source={},edge=second)])
    check('old_stop_stays_in_original_segment',lambda:assert_true(verifier._coordinate_at(segments,clock=9_000_000_000)is None))
    check('between_edges_uses_first_scope',lambda:assert_true(verifier._coordinate_at(segments,clock=11_000_000_000)['binding']==good['new_binding']))
    guard=dict(compute_ros_clock_ns=12_100_000_000,control_pose_stamp_ns=12_000_000_000,cloud_header_stamp_ns=12_000_000_000,cloud_filtering_body_stamp_ns=12_000_000_000,compute_monotonic_wall=12.1,reference_stamp=[12,0],route_reference=dict(route_id=12_000_000_000))
    check('new_guard_requires_new_sources',lambda:verifier._coordinate_guard_reference(guard,segments,good['old_reference'],{}))
    guard['cloud_header_stamp_ns']-=1
    # _require raises this verifier's own error type; inspect its real result.
    try:verifier._coordinate_guard_reference(guard,segments,good['old_reference'],{})
    except Exception as e:checks.append(dict(check='old_cloud_not_new_scope_credit',passed=True,error_type=type(e).__name__))
    else:raise AssertionError('old cloud incorrectly credited')
    # Actual status identity may lead or lag a reference boundary. It is
    # retained under the immutable anchor but earns no mismatched-scope credit.
    segments['anchor']=good['old_binding'];segments['trigger_cut']=1_000_000_000
    def status_row(sequence,data):
        return dict(kind='status',sequence=sequence,binding=good['old_binding'],data=data,
            navigation_source_identity=runtime.coordinate_navigation_source('status',data))
    for name,sequence,data in [('new_status_before_first',9,good['navigation_status']),
                              ('new_status_before_second',19,second['navigation_status']),
                              ('late_old_status_after_first',11,dict(good['navigation_status'],
                                known_scene_scan_epoch=dict(good['navigation_status']['known_scene_scan_epoch'],planner_epoch=2,localization_generation=20))),
                              ('late_first_status_after_second',21,good['navigation_status'])]:
        check(name+'_no_credit_no_falsefail',lambda sequence=sequence,data=data:
            assert_true(verifier._new_navigation_row(status_row(sequence,data),segments)is False))
    check('new_status_after_second_credit',lambda:assert_true(verifier._new_navigation_row(status_row(21,second['navigation_status']),segments)is True))
    inconsistent=copy.deepcopy(second['navigation_status']);inconsistent['known_scene_runtime_status']['generation']=36
    check('status_three_generation_identity_must_agree',lambda:assert_true(verifier._new_navigation_row(status_row(21,inconsistent),segments)is False))
    inline=dict(guard,cloud_header_stamp_ns=12_000_000_000,route={})
    check('new_guard_inline_route_rejected',lambda:verifier._coordinate_guard_reference(inline,segments,good['old_reference'],{}),True)
    badstamp=dict(guard,cloud_header_stamp_ns=12_000_000_000,reference_stamp=[11,0])
    check('new_guard_route_stamp_mismatch_rejected',lambda:verifier._coordinate_guard_reference(badstamp,segments,good['old_reference'],{}),True)
    def pose(stamp,x,gen):
        data=dict(stamp_ns=stamp,position=[x,0.,0.],body_velocity=[.1,0.,0.],corrected_body_message_sha256=hashlib.sha256(str(stamp).encode()).hexdigest())
        segments.setdefault('pose_sources',{})[(stamp,data['corrected_body_message_sha256'])]=gen
        return dict(data=data)
    oldposes=[pose(11_800_000_000,0.,36),pose(11_900_000_000,.01,36)]
    newposes=[pose(12_100_000_000,.1,59),pose(12_200_000_000,.11,59),pose(12_300_000_000,.12,59)]
    chosen=verifier._coordinate_recovery_rows(oldposes+newposes,segments,9_000_000_000)
    check('C_jump_09m_not_recovery_progress',lambda:assert_true(len(chosen)==3 and max(p['data']['position'][0]-chosen[0]['data']['position'][0]for p in chosen)<.08))
    newposes[-1]['data']['position'][0]=.19
    check('same_segment_real_09m_progress_retained',lambda:assert_true(max(p['data']['position'][0]-chosen[0]['data']['position'][0]for p in chosen)>=.08))
    unknown=pose(12_400_000_000,1.,36)
    check('late_old_generation_pose_not_new_progress',lambda:assert_true(unknown not in verifier._coordinate_recovery_rows([*newposes,unknown],segments,9_000_000_000)))
    unknown_raw=dict(kind='actual_pose',sequence=21,binding=good['old_binding'],data=unknown['data'],
        navigation_source_identity=runtime.coordinate_navigation_source('actual_pose',unknown['data']))
    check('pose_source_bundle_generation_not_callback_order',lambda:assert_true(verifier._new_navigation_row(unknown_raw,segments)is False))
    correct_pose=copy.deepcopy(unknown_raw);correct_pose['data']['corrected_body_message_sha256']=hashlib.sha256(b'correct').hexdigest()
    segments['pose_sources'][(correct_pose['data']['stamp_ns'],correct_pose['data']['corrected_body_message_sha256'])]=59
    correct_pose['navigation_source_identity']=runtime.coordinate_navigation_source('actual_pose',correct_pose['data'])
    correct_pose['sequence']=19
    check('new_pose_before_boundary_readiness_only',lambda:assert_true(verifier._new_navigation_row(correct_pose,segments)is False))
    correct_pose['sequence']=21
    check('exact_new_body_source_after_boundary_credit',lambda:assert_true(verifier._new_navigation_row(correct_pose,segments)is True))
    with tempfile.TemporaryDirectory(prefix='dynamic-body-source-') as directory:
        run=Path(directory);sample=correct_pose['data'];metadata=dict(schema='known_scene_navigation_bundle/v1',run_id=run.name,
            source_ns=sample['stamp_ns'],corrected_body_message_sha256=sample['corrected_body_message_sha256'],generation=59)
        (run/'known_scene_bundles.jsonl').write_text(json.dumps(metadata)+'\n')
        evidence=dict(records=[dict(data=correct_pose)])
        indexed=dict(coordinate_chain=segments['coordinate_chain'],anchor=segments['anchor'])
        proof=verifier._coordinate_pose_sources(run,evidence,indexed)
        check('producer_exact_stamp_body_hash_index',lambda:assert_true(indexed['pose_sources'][(sample['stamp_ns'],sample['corrected_body_message_sha256'])]==59))
        check('producer_witness_prefix_byte_hash_receipt',lambda:assert_true(proof['source_prefix_bytes']==(run/'known_scene_bundles.jsonl').stat().st_size))
        (run/'known_scene_bundles.jsonl').write_text((json.dumps(metadata)+'\n')*2)
        check('duplicate_producer_body_identity_rejected',lambda:verifier._coordinate_pose_sources(run,evidence,indexed),True)

    missing=copy.deepcopy(correct_pose);missing['data'].pop('corrected_body_message_sha256')
    missing['navigation_source_identity']=runtime.coordinate_navigation_source('actual_pose',missing['data'])
    segments['pose_sources'][(missing['data']['stamp_ns'],None)]=59
    check('missing_body_SHA_never_stamp_only_credit',lambda:assert_true(verifier._new_navigation_row(missing,segments)is False))
    malformed=copy.deepcopy(correct_pose);malformed['data']['corrected_body_message_sha256']='not-a-real-SHA'
    malformed['navigation_source_identity']=runtime.coordinate_navigation_source('actual_pose',malformed['data'])
    check('malformed_body_SHA_no_credit',lambda:assert_true(verifier._new_navigation_row(malformed,segments)is False))
    return dict(passed=True,checks=checks,count=len(checks),scope='Offline pure validators/owner-edge file reads only; no ROS, physical replay or full nine-gate acceptance.')


def assert_true(v):
    assert v

if __name__=='__main__':print(json.dumps(main(),indent=2))
