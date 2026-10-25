"""One startup source/selection receipt; no control or acceptance authority."""
import hashlib
import json
from pathlib import Path
import sys
import time
import os


def write_binding(node):
    if getattr(node,'_r86_prepared_mailbox',None) is None:
        return
    here=Path(__file__).resolve().parent
    run=Path(node.run).resolve()
    scope_path=run/'navigation_scope.json'
    scope=json.loads(scope_path.read_bytes())
    references=scope['references']
    paths={name:Path(sys.modules[name].__file__).resolve() for name in
        ('known_scene_inputs','r86_prepared_feedback','r86_stage_probe',
         'r87_diagnostic_probe','teacher_wrapper')}
    for name in ('r89_thread_probe','r89_runtime_trace','r96_control_cpu_probe','r74_cooperative_executor','r90_cooperative_executor'):
        paths[name]=Path(sys.modules[name].__file__).resolve()
    for name in ('r99_clock_feedback','r99_clock_probe'):
        paths[name]=Path(sys.modules[name].__file__).resolve()
    # Candidate-specific auxiliary modules are optional, but every actually
    # imported candidate module (including default-OFF imports) belongs to this scope.
    # Do not import missing modules or assume a direction-specific status API.
    for name in sorted(sys.modules):
        if name.startswith(('r102_', 'r103_', 'r104_', 'r105_', 'r106_', 'r107_', 'r109_', 'r111_', 'r114_', 'r118_')) or name in ('status_payload_handoff', 'imu_causal_buffer', 'bridge_safety_handoff'):
            source=getattr(sys.modules[name],'__file__',None)
            if source is None:
                raise RuntimeError('Candidate actual imported module has no source: '+name)
            paths[name]=Path(source).resolve()
    paths['shared_controller']=Path(sys.modules['teacher_wrapper'].base.__file__).resolve()
    paths['controller']=Path(sys.modules['__main__'].__file__).resolve()
    paths['r86_runtime_binding']=Path(__file__).resolve()
    heading_module=sys.modules.get('transition_gate')
    heading_update=getattr(getattr(node,'heading_gate',None),'update',None)
    heading_function=getattr(heading_update,'__func__',None)
    if (heading_module is None or heading_function is not heading_module.TeacherHeadingGate.update
            or Path(heading_function.__code__.co_filename).resolve()!=here/'transition_gate.py'):
        raise RuntimeError('Actual Teacher heading update is not the candidate transition_gate method')
    paths['transition_gate']=Path(heading_function.__code__.co_filename).resolve()
    if getattr(node,'_r91_empty_retry_yield',None) is not None:
        paths['r91_empty_retry_yield']=Path(sys.modules['r91_empty_retry_yield'].__file__).resolve()
    ready_commit=getattr(node,'_r92_ready_commit',None)
    if ready_commit is not None:
        paths['r92_ready_commit']=Path(sys.modules['r92_ready_commit'].__file__).resolve()
    bridge_handoff=getattr(node,'_bridge_safety_handoff',None)
    bridge_environment=os.environ.get('BRIDGE_SAFETY_INGRESS','0')
    if bridge_environment not in ('0','1') or (bridge_handoff is not None)!=(bridge_environment=='1'):
        raise RuntimeError('Bridge ingress actual instance/selection differs')
    bridge_selection=None if bridge_handoff is None else bridge_handoff.snapshot()
    if bridge_selection is not None and not all(bridge_selection.get(key) is True for key in
            ('same_control_group','ingress_group_is_r86','ingress_callback_bound_to_node')):
        raise RuntimeError('Bridge ingress actual callback/owner group binding differs')
    asynchronous=getattr(node,'_r88_admission',None)
    short_environment=os.environ.get('SCAN_SHORT_INTAKE','0')
    short_module=sys.modules.get('r88_async_admission')
    short_selections=(getattr(sys.modules['teacher_wrapper'],'SCAN_SHORT_INTAKE_ENABLED',None),
        getattr(short_module,'SHORT_INTAKE_ENABLED',None),getattr(asynchronous,'short_intake_enabled',None))
    if (short_environment not in ('0','1') or
            any(type(value)is not bool or value!=(short_environment=='1') for value in short_selections)):
        raise RuntimeError('SCAN short intake subscription/Admission/environment selection differs')
    short_snapshot=asynchronous.intake_snapshot()
    if short_environment=='1' and (not all(short_snapshot.get(k)is True for k in
            ('enabled','one_original_fifo','same_control_group','ingress_group_is_r86'))
            or short_snapshot['subscription_count']!=4 or short_snapshot['byte_limit']!=2097152
            or short_snapshot['receive_pressure_enabled']is not False):
        raise RuntimeError('Actual SCAN short intake groups/bounds/owner differ')
    short_selection=dict(environment_value=short_environment,module_enabled=short_selections[1],actual=short_snapshot)
    poll_environment=os.environ.get('R110_POLL_YIELD','0')
    poll_module=sys.modules.get('r88_async_admission')
    poll_module_enabled=getattr(poll_module,'POLL_YIELD_ENABLED',None)
    poll_instance_enabled=getattr(asynchronous,'poll_yield_enabled',None)
    if (poll_environment not in ('0','1') or asynchronous is None or
            type(poll_module_enabled)is not bool or type(poll_instance_enabled)is not bool or
            poll_module_enabled!=(poll_environment=='1') or poll_instance_enabled!=poll_module_enabled):
        raise RuntimeError('R110 actual module/Admission selection differs')
    poll_counter_names=('poll_yield_count','poll_yield_with_queue_count',
        'poll_yield_last_queue_count','poll_yield_last_queue_bytes')
    poll_counters={name:getattr(asynchronous,name,None) for name in poll_counter_names}
    if any(type(value)is not int or value<0 for value in poll_counters.values()):
        raise RuntimeError('R110 actual Admission counters unavailable')
    poll_selection=dict(environment_value=poll_environment,enabled=poll_module_enabled,
        actual_instance_enabled=poll_instance_enabled,before_executor_spin=True,
        process_fixed=True,default_off=True,initial_counters=poll_counters,
        exit_receipt='r88_async_physical_exit.json',exit_key='r110_poll_yield',
        subsequent_feedback_service_guaranteed=False)
    import inspect as _r114_inspect
    clock_service_environment=os.environ.get('R114_CLOCK_SERVICE','0')
    clock_service_module=sys.modules.get('r114_clock_service')
    clock_service_controller=sys.modules['__main__']
    clock_service_instance=getattr(node,'_r114_clock_service',None)
    clock_service_selections={name:getattr(module,'CLOCK_SERVICE_ENABLED',None)
        for name,module in (('r114_clock_service',clock_service_module),('controller',clock_service_controller))}
    if (clock_service_environment!='1' or
            any(type(value)is not bool or not value for value in clock_service_selections.values()) or
            clock_service_instance is None or type(clock_service_instance)is not clock_service_module.Service or
            clock_service_instance.enabled is not True or clock_service_instance.node is not node):
        raise RuntimeError('R114 actual module/controller/Service selection differs')
    paths['r114_clock_service']=Path(clock_service_module.__file__).resolve()
    clock_service_body=_r114_inspect.unwrap(node.known_scene_callbacks['body'])
    clock_service_hook=clock_service_controller.PIDNavigation.on_odom
    if (getattr(clock_service_body,'__self__',None)is not node or
            getattr(clock_service_body,'__func__',None)is not clock_service_hook or
            Path(clock_service_hook.__code__.co_filename).resolve()!=here/'controller.py' or
            not {'CLOCK_SERVICE_ENABLED','_r114_clock_service','service'}<=set(clock_service_hook.__code__.co_names)):
        raise RuntimeError('R114 actual R86 body dispatch is not the reviewed PID on_odom hook')
    clock_service_receipt=clock_service_instance.receipt()
    clock_service_counter_names={'calls','busy','empty','serviced','exceptions','shutdown','rows','evicted','diagnostic_errors'}
    if (clock_service_receipt.get('schema')!='r114_original_clock_service/v1' or
            clock_service_receipt.get('enabled')is not True or
            clock_service_receipt.get('environment_value')!=clock_service_environment or
            clock_service_receipt.get('process_fixed')is not True or
            clock_service_receipt.get('capacity')!=64 or
            set(clock_service_receipt.get('counts',{}))!=clock_service_counter_names or
            any(type(value)is not int or value<0 for value in clock_service_receipt['counts'].values())):
        raise RuntimeError('R114 actual startup Service receipt/counters differ')
    clock_service_selection=dict(environment_value=clock_service_environment,
        actual_module_selections=clock_service_selections,actual_instance_enabled=True,
        actual_body_hook='PIDNavigation.on_odom',actual_body_code_path=str(here/'controller.py'),
        before_executor_spin=True,process_fixed=True,default_off=True,
        initial_receipt=clock_service_receipt,exit_receipt='r114_clock_service.json',
        runtime_service_observed=False,recovery_validation_claimed=False)
    probe_take=sys.modules.get('r107_scan_take_probe')
    probe_transport=sys.modules.get('r107_scan_transport_probe')
    probe_environment=os.environ.get('R107_SCAN_TRANSPORT_PROBE','0')
    recorder=getattr(asynchronous,'transport_probe',None)
    scan_binding=getattr(probe_take,'_binding',None)
    install_receipt=getattr(node,'_r107_take_receipt',None)
    if (probe_environment not in ('0','1') or probe_take is None or probe_transport is None
            or (recorder is not None)!=(probe_environment=='1')
            or (scan_binding is not None)!=(probe_environment=='1')
            or not isinstance(install_receipt,dict)
            or install_receipt.get('installed')!=(probe_environment=='1')):
        raise RuntimeError('R118 actual R107 recorder/take installation differs')
    for name,module in (('r107_scan_take_probe',probe_take),('r107_scan_transport_probe',probe_transport)):
        paths[name]=Path(module.__file__).resolve()
    if probe_environment=='1':
        if (type(recorder)is not probe_transport.Recorder or scan_binding[0]is not clock_service_instance.executor
                or not clock_service_module._same_method(clock_service_instance.take,scan_binding[3])
                or not clock_service_module._same_method(clock_service_instance.executor._take_subscription,scan_binding[3])
                or not (run/'r107_scan_take_install.json').is_file()):
            raise RuntimeError('R118 R114 must capture final R107 take wrapper and available recorder')
    transport_receipt=dict(environment_value=probe_environment,actual_recorder=recorder is not None,
        actual_take_installed=scan_binding is not None,installation=install_receipt,
        service_uses_final_take=True,capacity=probe_transport.CAPACITY,
        width=probe_transport.WIDTH,
        buffer_bytes=0 if recorder is None else len(recorder.buffer)*recorder.buffer.itemsize,
        anomaly_buffer_bytes=0 if recorder is None else len(recorder.anomaly)*recorder.anomaly.itemsize,
        exit_receipt='r88_async_physical_exit.json',exit_key='r107_scan_transport',
        take_exit_key='r107_scan_take',trigger_observed=False)
    feedback_module=sys.modules['r118_feedback_evidence']
    feedback_environment=os.environ.get('R118_FEEDBACK_EVIDENCE','0')
    feedback_recorder=getattr(node._r86_prepared_mailbox,'evidence_probe',None)
    if (feedback_environment not in ('0','1')
            or (feedback_recorder is not None)!=(feedback_environment=='1')
            or feedback_module.RECORDER is not feedback_recorder
            or feedback_recorder is not None and type(feedback_recorder)is not feedback_module.Recorder):
        raise RuntimeError('R118 feedback evidence actual Mailbox recorder differs')
    feedback_evidence_selection=dict(environment_value=feedback_environment,
        actual_recorder=feedback_recorder is not None,before_executor_spin=True,
        retention='first256_tail1024_first_slow_pre512_post512_first_external_pre512_post512',
        buffer_bytes=0 if feedback_recorder is None else sum(len(a)*a.itemsize for a in
            (feedback_recorder.ring,feedback_recorder.window,feedback_recorder.external_window)),
        columns=feedback_module.COLUMNS,stages=feedback_module.STAGES,
        exit_receipt='r118_feedback_exit.json',trigger_observed=False)
    worker_probe=getattr(asynchronous.worker,'_r93_probe',None)
    worker_environment=os.environ.get('R93_HANDOFF_PROBE','0')
    if (worker_environment not in ('0','1') or (worker_probe is not None)!=(worker_environment=='1')):
        raise RuntimeError('R118 actual parent worker observer unavailable or differs')
    for name in ('r88_async_admission','r88_physical_worker','r93_worker_probe'):
        if name in sys.modules:paths[name]=Path(sys.modules[name].__file__).resolve()
    pressure_environment=os.environ.get('R116_RECEIVE_PRESSURE','0')
    pressure_module=getattr(poll_module,'RECEIVE_PRESSURE_ENABLED',None)
    pressure_instance=getattr(asynchronous,'receive_pressure_enabled',None)
    if (pressure_environment not in ('0','1') or
            pressure_environment=='1' and (pressure_module is not True or pressure_instance is not True)
            or pressure_environment=='0' and (pressure_module not in (None,False) or pressure_instance not in (None,False))):
        raise RuntimeError('R118 actual R116 module/Admission selection differs')
    pressure_counters={name:getattr(asynchronous,name,None) for name in
        ('receive_pressure_calls','receive_pressure_consumed','receive_pressure_still_rejected')}
    if pressure_environment=='1' and any(type(v)is not int or v<0 for v in pressure_counters.values()):
        raise RuntimeError('R118 actual R116 counters unavailable')
    pressure_selection=dict(environment_value=pressure_environment,actual_module=pressure_module,
        actual_instance=pressure_instance,initial_counters=pressure_counters,
        enabled=pressure_environment=='1',exit_receipt='r88_async_physical_exit.json',exit_key='r116_receive_pressure',
        trigger_observed=False,original_queue_bounds_preserved=True)
    source_environment=os.environ.get('R118_SOURCE_AGE_PROBE','0')
    source_module=sys.modules.get('r118_source_age_probe')
    source_instance=None if source_module is None else getattr(source_module,'PROBE',None)
    source_binding=getattr(node,'_r118_source_age_binding',None)
    if (source_environment not in ('0','1')
            or (source_instance is not None)!=(source_environment=='1')
            or (source_binding is not None)!=(source_environment=='1')
            or source_environment=='1' and (getattr(source_module,'OPTION',None)!=source_environment
                or source_instance.role!='controller' or source_instance.binding!=source_binding
                or source_binding.get('run')!=str(run))):
        raise RuntimeError('R118 actual controller source-age observer/binding differs')
    source_age_selection=dict(environment_value=source_environment,
        actual_controller_binding=source_binding,actual_recorder=source_instance is not None,
        controller_capacity=None if source_module is None else source_module.CAPACITY,
        post_capacity=None if source_module is None else source_module.POST,
        actual_bridge_reader_instances_claimed=False,trigger_observed=False,
        exit_receipts=['r118_source_age_controller.json','r118_source_age_bridge.json','r118_source_age_reader.json'])
    import r97_queue_budget
    paths['r97_queue_budget']=Path(r97_queue_budget.__file__).resolve()
    budget_selection=r97_queue_budget.selection(node.profile)
    if asynchronous is not None and (
            asynchronous.queue_budget_selection != budget_selection or
            asynchronous.queue_byte_limit != budget_selection['effective_bytes'] or
            (asynchronous.queue_budget is not None) != budget_selection['observation_enabled']):
        raise RuntimeError('R97 requested selection differs from actual Admission queue budget')
    if asynchronous is None and budget_selection['environment_value'] is not None:
        raise RuntimeError('R97 explicit queue budget requires actual R88 Admission')
    ready_poll_environment=os.environ.get('R94_SCAN_READY_POLL','0')
    ready_poll_enabled=False if asynchronous is None else getattr(asynchronous,'scan_ready_poll',False)
    if (ready_poll_environment not in ('0','1') or type(ready_poll_enabled) is not bool
            or ready_poll_enabled != (ready_poll_environment == '1')):
        raise RuntimeError('R94 requested flag differs from actual Admission.scan_ready_poll')
    cpu_probe=sys.modules['r96_control_cpu_probe']
    cpu_environment=os.environ.get('R96_CONTROL_CPU_PROBE','0')
    if cpu_environment not in ('0','1') or cpu_environment!=cpu_probe.OPTION:
        raise RuntimeError('R96 requested flag differs from imported probe selection')
    clock_probe=sys.modules['r99_clock_probe'];clock_environment=os.environ.get('R99_CLOCK_PROBE','0')
    interval_environment=os.environ.get('R102_CLOCK_READ_INTERVAL_PROBE','0')
    feedback_environment=os.environ.get('R99_CLOCK_FEEDBACK','0')
    feedback=getattr(node,'_r99_policy',None)
    if (clock_environment not in ('0','1') or clock_environment!=clock_probe.OPTION
            or feedback_environment not in ('0','1')
            or (feedback is not None)!=(feedback_environment=='1')
            or clock_probe.status()['installed']!=(clock_environment=='1')
            or interval_environment not in ('0','1')
            or interval_environment!=clock_probe.READ_OPTION
            or (interval_environment=='1' and (clock_environment!='1' or feedback_environment!='0'))):
        raise RuntimeError('R99 actual selection/installation differs')
    if asynchronous is not None:
        for name in ('r88_async_admission','r88_physical_worker','r85_physical_path_gate','r85_geometry_admission','pending_path_commit'):
            paths[name]=Path(sys.modules[name].__file__).resolve()
        if os.environ.get('R93_HANDOFF_PROBE', '0') == '1':
            paths['r93_worker_probe']=Path(sys.modules['r93_worker_probe'].__file__).resolve()
    imu_environment=os.environ.get('R103_IMU_SAFETY_INTAKE','0')
    imu_buffer=getattr(node,'imu_source_buffer',None)
    imu_intake=getattr(imu_buffer,'safety_intake',None)
    if imu_environment not in ('0','1') or (imu_intake is not None)!=(imu_environment=='1'):
        raise RuntimeError('R103 requested IMU intake differs from actual instance')
    imu_selection=dict(environment_value=imu_environment,enabled=imu_intake is not None,
        before_executor_spin=True,candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    if imu_intake is not None:
        imu_selection.update(original_validator_is_initial_raw=imu_intake.original_raw_imu is node.raw_imu,
            reading_view_distinct=imu_intake.reading_view is not imu_intake.original_raw_imu,
            fifo_validator_distinct=imu_buffer.validator is not imu_intake.original_raw_imu)
        if not all(imu_selection[k] for k in ('original_validator_is_initial_raw','reading_view_distinct','fifo_validator_distinct')):
            raise RuntimeError('R103 IMU validator ownership differs before spin')
    early_environment=os.environ.get('R103_EARLY_REQUEST_ACK','0')
    early_modules={name:sys.modules[name] for name in ('path_handoff','automatic_reference_runtime')}
    early_actual={name:getattr(module,'EARLY_REQUEST_ACK_ENABLED',None) for name,module in early_modules.items()}
    if (early_environment not in ('0','1') or any(type(value)is not bool or value!=(early_environment=='1')
            for value in early_actual.values())):
        raise RuntimeError('R103 early request completion selection differs across actual modules')
    for name,module in early_modules.items():paths[name]=Path(module.__file__).resolve()
    if (os.environ.get('R103_SCAN_SHARED_STORAGE','0')!='0'
            or getattr(asynchronous,'shared_storage',None) is not None
            or budget_selection['effective_bytes']!=2097152):
        raise RuntimeError('R103 handoff trial must preserve original logical queue policy')
    early_selection=dict(environment_value=early_environment,enabled=early_environment=='1',
        actual_module_selections=early_actual,default_off=True,process_fixed=True,
        successful_completion_only=True,original_global_counter_unchanged=True,
        ordinary_exhausted_path_retry_unchanged=True,shared_storage_enabled=False,
        candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    replacement_environment=os.environ.get('R103_EXHAUSTED_DRIVE_REPLACEMENT','0')
    replacement_actual=getattr(sys.modules['path_handoff'],'EXHAUSTED_DRIVE_REPLACEMENT_ENABLED',None)
    if (replacement_environment not in ('0','1') or type(replacement_actual)is not bool
            or replacement_actual!=(replacement_environment=='1')):
        raise RuntimeError('R103 exhausted drive replacement selection differs from actual module')
    replacement_selection=dict(environment_value=replacement_environment,enabled=replacement_actual,
        default_off=True,process_fixed=True,old_execution_samples_condition_preserved=True,
        new_curve_admission_and_transaction_preserved=True,shared_storage_enabled=False,
        candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    join_environment=os.environ.get('R104_PARKING_CAUSAL_JOIN','0')
    join_modules={name:sys.modules['__main__' if name=='controller' else name]
        for name in ('known_scene_inputs','controller','r104_parking_causal_join')}
    join_actual={name:getattr(module,'PARKING_CAUSAL_JOIN_ENABLED',None) for name,module in join_modules.items()}
    join_instance=getattr(node,'_r104_parking_causal_join',None)
    if (join_environment not in ('0','1') or any(type(value)is not bool or value!=(join_environment=='1')
            for value in join_actual.values()) or (join_instance is not None)!=(join_environment=='1')
            or join_instance is not None and getattr(join_instance,'enabled',None)is not True):
        raise RuntimeError('R104 requested parking causal join differs from actual modules/instance')
    if (getattr(node,'_r99_policy',None) is not None or getattr(node,'_r86_expired_prefix',False)
            or os.environ.get('R99_CLOCK_FEEDBACK','0')!='0'
            or sys.modules['r86_prepared_feedback'].FRAMES!=16
            or sys.modules['r86_prepared_feedback'].BYTES!=33554432):
        raise RuntimeError('R104 requires original R86 bounds and R99/S1 OFF')
    for name,module in join_modules.items():paths[name]=Path(module.__file__).resolve()
    join_selection=dict(environment_value=join_environment,enabled=join_instance is not None,
        actual_module_selections=join_actual,before_executor_spin=True,default_off=True,process_fixed=True,
        original_ready_frames=16,original_ready_bytes=33554432,old_zero_source_times_preserved=True,
        future_clock_wait_enabled=False,candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    terminal_environment=os.environ.get('R105_TERMINAL_PARKING_OWNER','0')
    terminal_modules={name:sys.modules['__main__' if name=='controller' else name]
        for name in ('known_scene_inputs','controller','r105_terminal_parking_owner')}
    terminal_actual={name:getattr(module,'TERMINAL_PARKING_OWNER_ENABLED',None) for name,module in terminal_modules.items()}
    if (terminal_environment not in ('0','1') or any(type(value)is not bool or value!=(terminal_environment=='1')
            for value in terminal_actual.values())
            or type(getattr(node,'_r105_terminal_owner_enabled',None))is not bool
            or node._r105_terminal_owner_enabled!=(terminal_environment=='1')):
        raise RuntimeError('R105 actual terminal policy differs across modules and node')
    import r85_resource_renewal as renewal
    resource_config=renewal.configuration(node.profile)
    if (os.environ.get('R105_RETAINED_BUDGET_170GB','0')!='1'
            or not resource_config or resource_config['maximum_retained_bytes']!=170000000000
            or resource_config['maximum_round_bytes']!=10000000000
            or resource_config['finish_reserve_s']!=300):
        raise RuntimeError('R105 actual resource policy differs from explicit new authorization')
    paths['r85_resource_renewal']=Path(renewal.__file__).resolve()
    for name,module in terminal_modules.items():paths[name]=Path(module.__file__).resolve()
    terminal_selection=dict(environment_value=terminal_environment,enabled=terminal_environment=='1',
        actual_module_selections=terminal_actual,instance_policy=node._r105_terminal_owner_enabled,
        before_executor_spin=True,default_off=True,process_fixed=True,
        candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'),resource_configuration=resource_config)
    prequalification_environment=os.environ.get('R106_PREQUALIFICATION_SOURCE_HOLD','0')
    prequalification_modules={name:sys.modules[name] for name in
        ('ordinary_parking','r106_prequalification_source_hold')}
    prequalification_actual={name:getattr(module,'PREQUALIFICATION_SOURCE_HOLD_ENABLED',None)
        for name,module in prequalification_modules.items()}
    if (prequalification_environment not in ('0','1') or any(type(value)is not bool
            or value!=(prequalification_environment=='1') for value in prequalification_actual.values())):
        raise RuntimeError('R106 actual source hold policy differs across modules')
    for name,module in prequalification_modules.items():paths[name]=Path(module.__file__).resolve()
    prequalification_selection=dict(environment_value=prequalification_environment,
        enabled=prequalification_environment=='1',actual_module_selections=prequalification_actual,
        before_executor_spin=True,default_off=True,process_fixed=True,
        actual_parking_instance_claimed=False,actual_branch_exercised=False,
        candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    speed_module=sys.modules['r109_speed_tiers']
    speed_selection=speed_module.configuration(node.profile)
    if speed_selection is None:
        raise RuntimeError('R117 explicit selected profile missing')
    paths['r109_speed_tiers']=Path(speed_module.__file__).resolve()
    speed_receipt=dict(selected_profile=speed_selection,before_executor_spin=True,
        actual_student_instance_claimed=False,
        student_actual_bounds_evidence='Existing Actor loaded-source metadata, same-frame PID and ACK records')
    records={}
    for name,path in paths.items():
        if path!=here/(name+'.py'):
            raise RuntimeError('R86 actual imported module path differs: '+name)
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if references.get(str(path))!=digest:
            raise RuntimeError('R86 actual imported module absent from frozen scope: '+name)
        records[name]={'path':str(path),'sha256':digest}
    value=dict(schema='r86_actual_controller_module_binding/v1',run=str(run),
        observed_monotonic_wall_ns=time.monotonic_ns(),modules=records,
        r102_observation_modules={name:records[name] for name in records if name.startswith('r102_')},
        r103_imu_safety_intake=imu_selection,
        r103_early_request_ack=early_selection,
        r103_exhausted_drive_replacement=replacement_selection,
        r104_parking_causal_join=join_selection,
        r105_terminal_parking_owner=terminal_selection,
        r106_prequalification_source_hold=prequalification_selection,
        r109_speed_tier=speed_receipt,
        r110_poll_yield=poll_selection,
        scan_short_intake=short_selection,
        ordinary_turn_control_handoff=dict(
            selection=node.profile.get('ordinary_turn_control_handoff',{'enabled':False}),
            actual_instance_enabled=node.heading_gate.continuous_turn_handoff_enabled),
        r114_clock_service=clock_service_selection,
        r107_scan_transport=transport_receipt,
        r118_feedback_evidence=feedback_evidence_selection,
        r116_receive_pressure=pressure_selection,
        r118_source_age_probe=source_age_selection,
        scope_sha256=hashlib.sha256(scope_path.read_bytes()).hexdigest(),
        bridge_safety_ingress=dict(environment_value=os.environ.get('BRIDGE_SAFETY_INGRESS','0'),
            enabled=getattr(node,'_bridge_safety_handoff',None) is not None,
            instance=bridge_selection),
        selection=node.profile['r86_prepared_feedback'],
        diagnostic_probe=dict(environment_key='R87_DIAGNOSTIC_PROBE',
            environment_value=os.environ.get('R87_DIAGNOSTIC_PROBE','0'),
            enabled=sys.modules['r87_diagnostic_probe'].enabled(),
            capacity=sys.modules['r87_diagnostic_probe'].CAPACITY,
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        intake_group_distinct_from_control=node._r86_intake_group is not node.default_callback_group,
        async_physical=dict(environment_key='R88_ASYNC_PHYSICAL',
            environment_value=os.environ.get('R88_ASYNC_PHYSICAL','0'),enabled=asynchronous is not None,
            worker_pid=None if asynchronous is None else asynchronous.worker.pid,
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        thread_probe=dict(environment_key='R89_THREAD_PROBE',
            environment_value=os.environ.get('R89_THREAD_PROBE','0'),
            status=sys.modules['r89_thread_probe'].status(),
            installation=getattr(node,'_r89_trace_receipt',None),
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        control_cpu_probe=dict(environment_key='R96_CONTROL_CPU_PROBE',
            environment_value=os.environ.get('R96_CONTROL_CPU_PROBE','0'),
            status=sys.modules['r96_control_cpu_probe'].status(),
            installation=getattr(node,'_r96_cpu_receipt',None),
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        clock_probe=dict(environment_key='R99_CLOCK_PROBE',environment_value=clock_environment,
            status=clock_probe.status(),installation=getattr(node,'_r99_clock_probe_receipt',None),
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        clock_read_interval_probe=dict(environment_key='R102_CLOCK_READ_INTERVAL_PROBE',
            environment_value=interval_environment, enabled=interval_environment=='1',
            actual_module=records['r99_clock_probe'],
            owner=clock_probe.status().get('observation',{}).get('owner'),
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        clock_feedback=dict(environment_key='R99_CLOCK_FEEDBACK',environment_value=feedback_environment,
            enabled=feedback is not None,timeout_ns=None if feedback is None else feedback.snapshot()['timeout_ns'],
            retry_period_wall_s=.05 if feedback is not None else None,
            timer_control_group=None if feedback is None else node._r99_timer.callback_group is node.default_callback_group,
            timer_autostart=False,source_receipts_refreshed=False,
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        eager_physical_send=dict(environment_key='R89_EAGER_PHYSICAL_SEND',
            environment_value=os.environ.get('R89_EAGER_PHYSICAL_SEND','0'),
            enabled=asynchronous is not None and asynchronous.worker.eager_submit),
        handoff_probe=dict(environment_key='R93_HANDOFF_PROBE',
            environment_value=os.environ.get('R93_HANDOFF_PROBE','0'),
            enabled=asynchronous is not None and
                getattr(getattr(asynchronous.worker,'_r93_probe',None),'enabled',False) is True,
            queue_observation_available=asynchronous is not None and
                getattr(asynchronous,'queue_probe_available',False) is True,
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        diagnostic_queue_budget=dict(budget_selection,
            actual_admission_enabled=asynchronous is not None,
            actual_observer_installed=asynchronous is not None and asynchronous.queue_budget is not None,
            actual_instance_byte_limit=None if asynchronous is None else asynchronous.queue_byte_limit),
        scan_ready_poll=dict(environment_key='R94_SCAN_READY_POLL',
            environment_value=ready_poll_environment,enabled=ready_poll_enabled,
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT')),
        positive_yield_executor=dict(getattr(node,'_r90_executor_receipt',{})),
        empty_retry_yield_executor=dict(getattr(node,'_r91_executor_receipt',{})),
        ready_commit=dict(environment_key='R92_READY_COMMIT',
            environment_value=os.environ.get('R92_READY_COMMIT','0'),
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'),
            enabled=ready_commit is not None,
            statistics=None if ready_commit is None else ready_commit.snapshot()),
        before_executor_spin=True,control_or_navigation_PASS=False)
    with (run/'r86_runtime_binding.json').open('x') as stream:
        json.dump(value,stream,indent=2);stream.write('\n')


def finish_empty_retry_yield(node,executor):
    """Exit-only receipt after owner stop and pool join; preserve existing outcome."""
    try:
        candidate=getattr(node,'_r91_empty_retry_yield',None)
        if candidate is None:
            return
        statistics=candidate.snapshot()
        spinner_native_tid=executor.r90_statistics()['spinner_native_tid']
        value=dict(schema='r91_empty_retry_yield_statistics/v1',
            selection=dict(node._r91_executor_receipt),statistics=statistics,
            spinner_native_tid=spinner_native_tid,
            owner_matches_spinner=statistics['owner_tid']==spinner_native_tid)
        (Path(node.run)/'r91_empty_retry_yield_statistics.json').write_text(
            json.dumps(value,allow_nan=False)+'\n')
    except Exception:
        # Missing/failed evidence is a verification failure, not a new shutdown result.
        try:node._r91_statistics_write_failed=True
        except Exception:pass
