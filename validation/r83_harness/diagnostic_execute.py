# Publication 2026-10-13: local paths redacted; numeric/control behavior retained; local admission is not transferred.
def execute(run,plan):
    import sys,datetime as execution_datetime
    TRACE_RUN['run']=run
    REVIEW=S
    sys.path.insert(0,str(REVIEW))
    from capture_owner_stop import OWNER_STOP_REASONS,publish_owner_stop,make_owner_stop,validate_owner_stop
    from pipeline_lifecycle import request_normal_stop
    from worker_archive_shutdown import await_archive_close,inspect_worker_archive
    from archive_close_policy import post_physics_archive_abort_reason
    from owned_guardian import Guardian,validated_members,ident as worker_archive_identity
    env=ros_environment(Path(plan['scan_workspace_contract']['workspace'])/'install/setup.bash');transport=json.loads((run/'cloud_transport_manifest.json').read_text())
    for k in transport['remove_environment_keys']:env.pop(k,None)
    env.update(transport['environment'])
    for k in plan['navigation_stack_diagnostic_environment']:
        env.pop(k,None)  # Diagnostic settings belong only to navigation_stack.
    sock='/tmp/teacher_closed_loop_'+uuid.uuid4().hex[:12]+'.sock'
    env.update(ROS_DOMAIN_ID=str(plan['ros_domain']),GZ_IP='127.0.0.1',GZ_PARTITION='teacher_closed_loop_'+run.name,
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'simulation/build'),TEACHER_SOCKET=sock,TEACHER_ACTUATOR_LOG=str(run/'actuator.jsonl'),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',ROS_LOG_DIR=str(run/'ros_logs'))
    env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('GALLIUM_DRIVER',None)
    selected_keys=('ROS_DOMAIN_ID','ROS_LOG_DIR','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION',
                   'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','GZ_PARTITION','GZ_IP','DISPLAY',
                   'LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS',
                   'AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH')
    write_new(run/'effective_environment.json',{k:env.get(k)for k in selected_keys})
    from storage_guard import observe as storage_observe, Watchdog
    before=resource();before['storage']=storage_observe(run,plan['profile'],starting=True,count_files=True);write_new(run/'resources_before.json',before)
    children={};identities={};handles=[];cleanup={};error=None;sensors_ready=False;natural_timeout=False;loaded_slam=None;loaded_scan=None;guardian=None;sample_complete=False;startup_stop=None;native_last_ns=0;native_tail=None
    acquisition_started=time.monotonic();wall_deadline=acquisition_started+plan['wall_budget_s']
    stream=(run/'owned_resources.jsonl').open('x',buffering=1)
    cleanup_errors=[]
    def attempt(label,operation,fallback=None):
        nonlocal error
        try:return operation()
        except BaseException as e:
            detail=label+': '+type(e).__name__+': '+str(e)
            cleanup_errors.append(detail);error=(error+'; 'if error else'')+detail
            return fallback
    def record(path,value):return attempt('receipt '+path.name,lambda:write_new(path,value))
    def safe_hash(path):return attempt('final hash '+path.name,lambda:sha(path))
    def start(role,command,child_env=None):
        deadline=execution_datetime.datetime.fromisoformat(json.loads((S/'AUTHORITY.json').read_text())['deadline_UTC'])
        if execution_datetime.datetime.now(execution_datetime.timezone.utc)>=deadline:raise RuntimeError('Overnight deadline prevents new owned role '+role)
        log=(run/(role+'.log')).open('x');handles.append(log)
        child,saved=guardian.start_role(role,command,env if child_env is None else child_env,log)
        children[role]=child;identities[role]=saved
        if identities[role]is None:raise RuntimeError('Could not bind owned process identity '+role)
        return child
    def interrupted(sig,frame):raise InterruptedError('Runner interrupted; cleanup is restricted to saved owned process groups')
    old_handlers={s:signal.signal(s,interrupted)for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    watchdog=None
    try:
        watchdog=Watchdog(run,plan['profile'],lambda:os.kill(os.getpid(),signal.SIGTERM)).start()
        guardian=Guardian(run,dict(plan,wall_budget_s=max(1.,wall_deadline-time.monotonic())),module_path=S/"owned_guardian.py")
        resource_guard(before)
        require_private_domain(plan['ros_domain'])
        scope=module(HERE/'pid_scope.py','closed_loop_execute_'+uuid.uuid4().hex)
        scope.verify_scope(run/'navigation_scope.json')
        if sha(run/'source_manifest.json')!=plan['source_manifest_sha256']or sha(run/'world.sdf')!=plan['world_sha256']:
            raise RuntimeError('Run preparation changed before execution')
        worker=start('worker',[plan['actor_python'],'-B',str(HERE/'ipc_request_window.py'),'--run',str(run),'--socket',sock],{**env,'CUDA_VISIBLE_DEVICES':''})
        deadline=time.monotonic()+30
        while not(run/'worker_ready').exists():
            if worker.poll()is not None:raise RuntimeError('CPU Actor exited before native IPC readiness')
            if time.monotonic()>deadline:raise TimeoutError('CPU Actor readiness exceeded30wall seconds')
            time.sleep(.05)
        start('bridge',[str(BRIDGE),'--ros-args','-r','__node:=ros_gz_bridge','-p','config_file:='+str(run/'sensor_bridge.yaml')])
        start('capture',[str(SYSTEM_PYTHON),'-B',str(REVIEW/'capture_candidate.py'),'--owner-diagnostic-stop','--run',str(run),
                         '--topic','/demo/teacher/overview','--archive-stride',str(plan['profile']['engineering_recording']['overview_archive_stride'])])
        navigation_env={**env,'DEMO_RUN_DIR':str(run),**plan['navigation_stack_diagnostic_environment']}
        write_new(run/'navigation_stack_effective_environment.json',{
            k:navigation_env.get(k)for k in (*selected_keys,'DEMO_RUN_DIR',*plan['navigation_stack_diagnostic_environment'])})
        start('navigation_stack',['ros2','launch',str(HERE/'stack.launch.py'),'run_dir:='+str(run)],navigation_env)
        if plan.get('bounded_shadow_sampling'):
            capture=plan['bounded_shadow_sampling']
            if sha(capture['contract_path'])!=capture['contract_sha256']or sha(HERE/'bounded_shadow_capture.py')!=capture['collector_source_sha256']:
                raise RuntimeError('Bounded sampling input changed')
            start('shadow_sampler',[str(SYSTEM_PYTHON),'-B',str(HERE/'bounded_shadow_capture.py'),'--run',str(run),'--contract',capture['contract_path']],navigation_env)
        time.sleep(.5)
        if any(p.poll()is not None for p in children.values()):raise RuntimeError('Owned dependency exited before physics started')
        loaded_slam=verify_loaded_slam(run,plan,identities['navigation_stack'])
        loaded_scan=verify_loaded_scan(run,plan,identities['navigation_stack'])
        if any(p.poll()is not None for p in children.values()):raise RuntimeError('Owned dependency exited during V7 binary witness')
        write_new(run/'R50_GAZEBO_CORRECTION_ENVIRONMENT.json',dict(LD_PRELOAD=str(S/'libgo2_ray_observer.so'),GO2_RECTIFY_RAYS='1',GO2_RECTIFY_UV='./multifloor_demo/teacher_mode/runs/20261011T074800Z_closed_loop_R48_actual_ray_geometry_90b4/ray_geometry_observation/sample_uv_rgba32f.bin',GO2_RECTIFY_DIRECTIONS='./local_evidence/go2_engineering_v33/R48_ACTUAL_RAY_GEOMETRY_ALIAS_FIX_STAGE/sampled_directions.f32',applies_only_to='owned Gazebo process',no_truth_navigation=True))
        (run/'ray_geometry_observation').mkdir()
        gazebo=start('gazebo',['gz','sim','-s','-r',str(run/'world.sdf')],{**env,'LD_PRELOAD':str(S/'libgo2_ray_observer.so'),'GO2_RAY_DIAG_DIR':str(run/'ray_geometry_observation'),'GO2_RECTIFY_RAYS':'1','GO2_RECTIFY_UV':'./multifloor_demo/teacher_mode/runs/20261011T074800Z_closed_loop_R48_actual_ray_geometry_90b4/ray_geometry_observation/sample_uv_rgba32f.bin','GO2_RECTIFY_DIRECTIONS':'./local_evidence/go2_engineering_v33/R48_ACTUAL_RAY_GEOMETRY_ALIAS_FIX_STAGE/sampled_directions.f32'})
        from known_scene_witness import record as record_known_scene_witness
        verify_loaded_native(run,plan,identities['gazebo'])
        from mission46_runtime_evidence import FileTail,native_projection,native_ns
        from startup_trace import startup_stop_decision
        from storage_guard import usage
        native_tail=FileTail(run/'telemetry.jsonl',project=native_projection)
        record_known_scene_witness(run,plan,identities,HERE,SLAM_WORKSPACE)
        write_new(run/'owned_processes_started.json',{role:{**identities[role],'role':role}for role in children})
        camera_deadline=time.monotonic()+30;last_resource=-math.inf;last_storage=-math.inf
        while worker.poll()is None:
            now=time.monotonic()
            for role,child in children.items():
                if role=='shadow_sampler':
                    rc=child.poll()
                    if rc==0:
                        result=json.loads((run/'bounded_shadow_samples/RESULT.json').read_text())
                        if result.get('complete')is not True:raise RuntimeError('Sampler exit without complete receipt')
                        sample_complete=True
                    elif rc is not None:raise RuntimeError('Bounded shadow sampler incomplete; inspect its receipt')
                elif role not in ('worker','gazebo')and child.poll()is not None:raise RuntimeError(role+' exited while Actor remained active')
            if sample_complete:break
            from first_goal_stop import observe_native_tail,first_goal_runtime_decision
            observed_native=observe_native_tail(native_tail,native_last_ns)
            native_last_ns=observed_native['native_ns']
            current_run_storage=usage([run])
            decision=first_goal_runtime_decision(run,observed_native,
                now-(wall_deadline-plan['wall_budget_s']),max(current_run_storage['allocated_bytes'],current_run_storage['logical_bytes']))
            if not decision['stop']:
                extra=diagnostic_stop(run,native_last_ns,now-acquisition_started,max(current_run_storage['allocated_bytes'],current_run_storage['logical_bytes']),budget_warning=bool(watchdog and watchdog.last and watchdog.last.get('R77_round_budget',{}).get('predictive_soft_stop')))
                if extra and watchdog and watchdog.last:extra['round_budget']=watchdog.last.get('R77_round_budget')
                if extra:decision=extra
            if decision['stop']:
                startup_stop=decision
                stop_record=dict(decision,actual_native_ns=native_last_ns,actual_monotonic_wall=now)
                write_new(run/'R33_DIAGNOSTIC_STOP.json',stop_record)
                break
            sensors_ready=(run/'frame_source.json').exists()
            if now>camera_deadline and not sensors_ready:raise TimeoutError('Actual overview camera failed to publish in30wall seconds')
            if now-last_resource>=2:
                sample=resource([p.pid for p in children.values()if p.poll()is None])
                count_storage=now-last_storage>=1
                sample['storage']=storage_observe(run,plan['profile'],count_files=count_storage)
                if count_storage:last_storage=now
                stream.write(json.dumps(sample)+'\n');resource_guard(sample);last_resource=now
            if gazebo.poll()is not None:
                worker.wait(timeout=10);break
            if now>wall_deadline:raise TimeoutError('Owned closed-loop run exceeded frozen wall budget')
            time.sleep(.2)
        if startup_stop is None and not sample_complete:
            from terminal_original_decision import terminal_original_decision
            terminal_storage=usage([run])
            observed_native,terminal=terminal_original_decision(run,native_tail,native_last_ns,
                time.monotonic()-acquisition_started,max(terminal_storage['allocated_bytes'],terminal_storage['logical_bytes']))
            native_last_ns=observed_native['native_ns']
            if terminal['stop']:
                startup_stop=terminal
                write_new(run/'R33_DIAGNOSTIC_STOP.json',dict(terminal,actual_native_ns=native_last_ns,
                    actual_monotonic_wall=time.monotonic(),observation_phase='after_worker_EOF'))
        if startup_stop:
            error='R71 stopped: '+startup_stop['reason']+'; strict full navigation UNVERIFIED'
        elif sample_complete:
            error='Bounded shadow data collected; navigation horizon intentionally interrupted, no navigation PASS'
        else:
            if worker.returncode!=0:raise RuntimeError('CPU Teacher worker returned nonzero')
            result=json.loads((run/'worker_result.json').read_text())
            if result.get('fault')is not None:error='Actor/native physical fault: '+str(result['fault'])
            try:gazebo.wait(timeout=8)
            except subprocess.TimeoutExpired:natural_timeout=True;error=error or 'Gazebo failed natural exit after done/damping'
    except Exception as e:error=f'{type(e).__name__}: {e}'
    finally:
        for s in old_handlers:signal.signal(s,signal.SIG_IGN)
        if guardian:
            for role,(child,saved)in guardian.children.items():
                children.setdefault(role,child);identities.setdefault(role,saved)
        typed_stop_error=None;typed_stop_kind=None
        try:
            from capture_owner_stop import make_owner_stop,publish_owner_stop
            reason=startup_stop['reason'] if startup_stop else ('R77_exceptional_stop'if error else'R77_original_run_terminated')
            evidence={}
            if reason in ('actual_original_mission_failed','actual_original_navigation_failed'):
                failure_file=run/('mission46_status.json'if reason=='actual_original_mission_failed'else'navigation_status.json')
                raw=failure_file.read_bytes();failure=json.loads(raw)
                evidence['original_failure_evidence']=dict(source_file=str(failure_file),sha256=__import__('hashlib').sha256(raw).hexdigest(),state=failure.get('stage',failure.get('state')))
            typed=make_owner_stop(reason,int(native_last_ns),time.monotonic(),**evidence)
            typed_stop_kind=typed['stop_kind']
            publish_owner_stop(run,typed)
        except BaseException as e:
            typed_stop_error=type(e).__name__+': '+str(e)
            cleanup_errors.append('typed_stop: '+typed_stop_error)
            error=(error+'; 'if error else'')+'typed_capture_stop_failed: '+typed_stop_error
        try:pipeline_stop=request_normal_stop(run,loaded_slam,identities.get('navigation_stack',{}),identity)
        except BaseException as e:
            pipeline_stop=dict(normal_completed=False,error=type(e).__name__+': '+str(e));cleanup_errors.append('pipeline: '+pipeline_stop['error'])
        if not pipeline_stop['normal_completed']:error=error or 'V19 normal pipeline drain failed: '+str(pipeline_stop['error'])
        if pipeline_stop.get('normal_completed'):
            capture_child=children.get('capture')
            capture_wait_start=time.monotonic()
            capture_wait_limit=min(wall_deadline,capture_wait_start+0.5)
            while (capture_child is not None and capture_child.poll() is None
                    and time.monotonic()<capture_wait_limit):
                if watchdog and watchdog.error or guardian and guardian.error:
                    break
                time.sleep(0.01)
            record(run/'capture_owner_wait.json',dict(
                maximum_grace_s=0.5,sent_signals=[],
                elapsed_s=time.monotonic()-capture_wait_start,
                returncode=capture_child.poll() if capture_child is not None else None))
        for role in ('shadow_sampler','navigation_stack','capture','bridge','gazebo'):
            try:cleanup[role]=stop_owned(children.get(role),identities.get(role),parent_first=role=='navigation_stack')
            except BaseException as e:
                cleanup[role]=dict(error=type(e).__name__+': '+str(e));cleanup_errors.append('owned cleanup '+role+': '+str(e));error=error or 'owned cleanup failed: '+role
        # Native physics is stopped before waiting for the unmodified Actor's
        # normal EOF/final archive; hard gates and original escalation remain.
        def archive_abort_reason():
            # Called only inside the existing helper, after owned physics stop
            # was confirmed. Failed receipts stay failed in the manifest.
            return post_physics_archive_abort_reason(
                cleanup, watchdog_error=watchdog.error if watchdog else None,
                guardian_error=guardian.error if guardian else None,
                now=time.monotonic(), wall_deadline=wall_deadline)
        try:
            worker_archive_close=await_archive_close(children.get('worker'),identities.get('worker'),
                cleanup.get('gazebo',{}),worker_archive_identity,validated_members,abort_reason=archive_abort_reason)
        except BaseException as e:
            worker_archive_close=dict(error=type(e).__name__+': '+str(e),complete=False)
            cleanup_errors.append('Actor archive close: '+str(e));error=error or 'Actor archive close failed'
        try:cleanup['worker']=stop_owned(children.get('worker'),identities.get('worker'))
        except BaseException as e:
            cleanup['worker']=dict(error=type(e).__name__+': '+str(e));cleanup_errors.append('Actor cleanup: '+str(e));error=error or 'Actor cleanup failed'
        try:worker_archive_integrity=inspect_worker_archive(run)if'worker'in children else None
        except BaseException as e:
            worker_archive_integrity=dict(verified=False,error=type(e).__name__+': '+str(e));cleanup_errors.append('Actor integrity: '+str(e))
        if worker_archive_integrity is not None and not worker_archive_integrity['verified']:
            error=error or 'Worker archive incomplete: '+str(worker_archive_integrity['error'])
        record(run/'worker_archive_close.json',worker_archive_close)
        record(run/'worker_archive_integrity.json',worker_archive_integrity)
        attempt('socket cleanup',lambda:Path(sock).unlink(missing_ok=True));attempt('resources stream close',stream.close)
        for i,f in enumerate(handles):attempt('owned log close '+str(i),f.close)
        try:guardian_receipt=guardian.close()if guardian else None
        except BaseException as e:
            guardian_receipt=dict(error=type(e).__name__+': '+str(e));cleanup_errors.append('guardian release: '+str(e));error=error or 'owned guardian release failed'
        if guardian_receipt is not None and guardian_receipt.get('error') is not None:
            detail='owned guardian reported failure: '+str(guardian_receipt['error'])
            cleanup_errors.append(detail);error=(error+'; 'if error else'')+detail
        if watchdog:attempt('watchdog close',watchdog.close)
        for sig,handler in old_handlers.items():attempt('signal handler restore '+str(sig),lambda sig=sig,handler=handler:signal.signal(sig,handler))
        native_guard_receipt=attempt('native guard receipt',lambda:json.loads((run/'R33_native_loaded_guard.json').read_text()),{})
        actual=[{'role':role,**(identities[role]or{'pid':child.pid,'identity_unavailable':True}),'returncode':child.returncode}for role,child in children.items()]
        child_proof=attempt('launch children receipt',lambda:launch_child_receipt(run/'navigation_stack.log',expected=plan['expected_launch_children']),dict(all_expected_children_clean=False,error='receipt failed'))
        roles_complete=set(children)==set((*REQUIRED_ROLES,*(['shadow_sampler']if plan.get('bounded_shadow_sampling')else[])))
        clean=roles_complete and all(x['returncode']==0 for x in actual)and child_proof['all_expected_children_clean']and all(not x.get('remaining_owned_group_members')and not x.get('identity_mismatch')and not x.get('identity_unavailable')for x in cleanup.values())
        if not clean:error=error or 'Owned role/launch child cleanup incomplete or nonzero'
        manifest={**plan,'cleanup_errors':cleanup_errors,'typed_stop_kind':typed_stop_kind,'typed_stop_error':typed_stop_error,'error':error,'owned_processes':actual,'cleanup':cleanup,
             'independent_owned_guardian':guardian_receipt,'bounded_shadow_sampling_complete':sample_complete,
             'R33_actual_diagnostic_stop':startup_stop,'R33_last_actual_native_ns':native_last_ns,
             'worker_archive_close':worker_archive_close,'worker_archive_integrity':worker_archive_integrity,
            'slam_loaded_binary_verified':bool(loaded_slam and loaded_slam.get('verified')),
            'slam_loaded_binary_receipt_sha256':safe_hash(run/'slam_loaded_binary.json')if(run/'slam_loaded_binary.json').is_file()else None,
            'scan_loaded_binary_verified':bool(loaded_scan and loaded_scan.get('verified')),
            'scan_loaded_binary_receipt_sha256':safe_hash(run/'scan_loaded_binary.json')if(run/'scan_loaded_binary.json').is_file()else None,
            'pipeline_normal_stop_sha256':safe_hash(run/'pipeline_normal_stop.json')if(run/'pipeline_normal_stop.json').exists()else None,
            'pipeline_normal_completed':pipeline_stop['normal_completed'],
            'required_owned_roles_complete':roles_complete,'all_owned_and_children_clean':clean,
            'launch_child_exit_evidence':child_proof,'gazebo_partition':env['GZ_PARTITION'],
            'native_plugin_sha256':plan['native_plugin_sha256'],'plugin_sha256':plan['native_plugin_sha256'],
             'protected_production_native_plugin_sha256':sha(ROOT/'simulation/build/libteacher_actuator.so'),
             'R33_native_loaded_guard_verified':(run/'R33_native_loaded_guard.json').is_file() and native_guard_receipt.get('verified') is True,
             'R33_native_loaded_guard_receipt_sha256':safe_hash(run/'R33_native_loaded_guard.json') if (run/'R33_native_loaded_guard.json').is_file() else None,
            'source_manifest_sha256':safe_hash(run/'source_manifest.json'),'runtime_plan_sha256':safe_hash(run/'runtime_plan.json'),
            'gazebo_natural_exit_grace_expired':natural_timeout,'actual_sensors_ready':sensors_ready,
            'runtime_started':bool(children),'runtime_status':'failed'if error else'completed_requires_independent_navigation_evaluation',
            'training_processes_signaled':False,'actor_started':'worker'in children,'navigation_controller_kind':'teacher',
            'high_level_source':'Actual SLAM body pose and causal IMU gyro, actual registered cloud and SCAN checked path',
            'truth_navigation_used':False,'actual_feedback_hz':'Measured from unique source headers; nominal camera-driven source '+str(plan['profile']['sensor_sampling']['camera_hz'])+'Hz; unchanged20Hz wall ticker is not new feedback'}
        if watchdog:
            manifest['storage_watchdog_error']=watchdog.error
            manifest['storage_last_observation']=watchdog.last
            if watchdog.error:
                manifest['error']=error=error or watchdog.error
                manifest['runtime_status']='failed'
        try:
            cap=json.loads((run/'capture_normal_stop.json').read_text()) if(run/'capture_normal_stop.json').exists()else None
            valid_capture=cap is not None and validate_owner_stop(cap)==json.loads((run/'CAPTURE_OWNER_STOP.json').read_text())
        except (OSError,ValueError,RuntimeError,KeyError,TypeError):valid_capture=False
        manifest['typed_capture_close_verified']=valid_capture and children.get('capture')is not None and children['capture'].returncode==0 and not(run/'capture_first_exception.json').exists()
        manifest['R77_full_autonomous_navigation_verified']=False
        manifest['R77_runtime_closure_completed']=(error is None and startup_stop is None
            and manifest['typed_capture_close_verified']and pipeline_stop['normal_completed']and clean
            and worker_archive_integrity is not None and worker_archive_integrity.get('verified')is True
            and guardian_receipt is not None and guardian_receipt.get('returncode')==0 and guardian_receipt.get('error') is None
            and manifest['slam_loaded_binary_verified']and manifest['scan_loaded_binary_verified']and manifest['R33_native_loaded_guard_verified']
            and not cleanup_errors and (watchdog is None or watchdog.error is None))
        record(run/'runtime_manifest.json',manifest)
        attempt('resources after',lambda:write_new(run/'resources_after.json',resource()))
        record(run/'summary_closed_loop_navigation.json',{'schema':'teacher_closed_loop_runtime_summary/v1',
            'status':'failed'if error else'unverified','run':str(run),'runtime_error':error,
            'levels':{'runtime':'failed'if error else'completed','navigation':'unverified','multifloor':'unverified','Sim2Sim':'unverified','real_robot':'unverified'},
            'scope_sha256':plan['scope_sha256'],'reason':'Original-source/SCAN/arrival/active-hold/native offline independent evaluation required',
            'compatibility_stand_window_diagnostic_is_navigation_acceptance':False})
    return error
