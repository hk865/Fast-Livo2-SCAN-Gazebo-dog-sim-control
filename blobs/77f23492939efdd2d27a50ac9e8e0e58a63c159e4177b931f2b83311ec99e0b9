"""Preview replacement and request overlap; never change control mathematics."""
import copy
import math
import time
import numpy as np

def context(self):
    active=self.path_admission_active
    inputs=getattr(self,'known_scene_inputs',None)
    cascade=getattr(self,'cascade',None)
    receipt=getattr(self,'path_receipt',None)
    if not isinstance(inputs,dict) or cascade is None or not isinstance(receipt,dict):
        return dict(allowed=False,conditions=dict(context_present=False),generation=None,coverage=None)
    generation=inputs.get('generation')
    coverage=None if self.continuous_route is None else self.continuous_route.next_index
    conditions=dict(source_generation_ready=self._known_generation_ready(),
        same_correction_generation=active is not None and active.get('correction_generation')==generation,
        same_coverage=active is not None and active.get('ordered_coverage_index')==coverage,
        same_cascade_goal=self.cascade_key==(self.request_id,self.waypoint_index),
        receipt_present=self.path_receipt is not None,
        same_path=self.path_receipt is not None and self.cascade.path_id==self.path_receipt['path_id'],
        same_trajectory=self.path_receipt is not None and self.path_receipt['trajectory_id']==self.active_trajectory_id)
    conditions.update(self.path_execution_eligibility()['conditions'])
    return dict(allowed=all(conditions.values()),conditions=conditions,generation=generation,coverage=coverage)

def preview(self,samples,value,metadata):
    required=(self.heading_gate.phase=='drive' and self.cascade is not None
        and self.cascade.capture_stamp is None and self.path_admission_active is not None)
    if not required:return dict(required=False,allowed=True)
    eligibility=context(self)
    if not eligibility['allowed']:
        return dict(required=True,allowed=False,reason='handoff_active_context_not_ready',eligibility=eligibility)
    points=np.asarray(samples,dtype=float);indices=[0]
    for i in range(1,len(points)):
        if np.linalg.norm(points[i,:2]-points[indices[-1],:2])>=1e-5:indices.append(i)
    pose=self.pose.copy();yaw=math.atan2(self.rotation[1,0],self.rotation[0,0])
    old=self.cascade.preview_reference(pose);candidate=copy.deepcopy(self.cascade)
    stamp=metadata['reference_stamp'];candidate.set_path(points[indices].tolist(),
        'handoff_preview:'+str(value['traj_id']),int(stamp[0])*1_000_000_000+int(stamp[1]),
        time.monotonic_ns(),'camera_init',position=pose)
    new=candidate.preview_reference(pose)
    wrap=lambda x:math.atan2(math.sin(x),math.cos(x))
    body_error=wrap(new['heading_rad']-yaw);change=wrap(new['heading_rad']-old['heading_rad'])
    if not all(map(math.isfinite,(yaw,body_error,change))):raise ValueError('Nonfinite handoff heading')
    limit=self.profile['teacher_transition']['drive_heading_max_rad']
    allowed=abs(body_error)<=limit and abs(change)<=limit
    return dict(required=True,allowed=allowed,reason='handoff_ready' if allowed else 'handoff_not_ready',
        body_heading_error_rad=body_error,reference_change_rad=change,maximum_error_rad=limit,
        old_reference=old,new_reference=new,eligibility=eligibility,
        source_pose_stamp_ns=int(self.pose_stamp),source_cloud_stamp_ns=int(self.cloud_stamp),preview_only=True)

def early_request(self,reference):
    if (self.heading_gate.phase!='drive' or self.cascade.capture_stamp is not None
        or not context(self)['allowed']):return None
    remaining=float(reference['remaining_m']);arc=float(self.cascade.spatial_config['arc_length_m'])
    if not 0<remaining<=arc:return None
    # A curve ending at the immutable goal needs its original arrival/capture.
    if reference['endpoint_is_fixed_goal']:return None
    key=(self.request_id,self.waypoint_index,self.known_scene_inputs['generation'],
        None if self.continuous_route is None else self.continuous_route.next_index,self.cascade.path_id)
    if self.path_handoff_early_requested_for==key:return None
    association=self.trajectory_association
    if association.reference_stamp is not None and association.accepted is None:return None
    before=self.reference_requests;self.request_plan()
    requested=self.reference_requests>before
    if requested:self.path_handoff_early_requested_for=key
    receipt=dict(schema='causal_path_handoff_early_request/v1',path_id=self.cascade.path_id,
        requested=requested,remaining_m=remaining,original_preview_arc_m=arc,
        source_pose_stamp_ns=int(self.pose_stamp),actual_source_times_refreshed=False,
        navigation_ground_truth_used=False)
    self.path_handoff_last_check=receipt
    self.evidence.append(self.run/'path_handoff_early_requests.jsonl',receipt)
    return receipt
