"""130GB current-retained shared cap plus unchanged conservative10GB round cap.

Historical cumulative counters remain statistics and never constrain shared
capacity. The per-round positive-growth channels remain monotonic and do not
refund deletions. Only the current owned runner may receive a stop request.
"""
from pathlib import Path
import hashlib,json,os,threading,time
from retained_usage import usage,current,B
S=Path(__file__).resolve().parent
SESSION=B/'STORAGE_SESSION.json'
POLICY=B/'CURRENT_STORAGE_BUDGET_POLICY.json'
HARD_BYTES=130_000_000_000
SOFT_BYTES=HARD_BYTES-536_870_912
ROUND_HARD_BYTES=10_000_000_000
MIN_RATE=17*1024**2;GRACE_S=122.;ATOMIC_BYTES=134_217_728;CLEANUP_BYTES=536_870_912

def capacity(path):
    p=Path(path);st=os.statvfs(p)
    return dict(path=str(p),device=os.stat(p).st_dev,available_bytes=st.f_bavail*st.f_frsize)

def register(run):
    value=str(Path(run).resolve());d=json.loads(SESSION.read_text())
    for key in ('roots','retained_roots'):
        d.setdefault(key,[])
        if value not in d[key]:d[key].append(value)
    d['runtime_active_policy_file']=str(POLICY)
    tmp=SESSION.with_suffix('.R80.tmp');tmp.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n');tmp.replace(SESSION)

class StorageStop(RuntimeError):
    def __init__(self,observation):
        self.observation=observation
        super().__init__('R80 current retained/round storage stop: '+', '.join(observation['stop_reasons']))

def round_evaluate(snapshot,baseline,run_usage,engineering,combined,previous=None):
    old=previous or {}
    counters={}
    baseline_totals=baseline['totals']
    for channel,key,baseline_key in [('new_logical_increments','logical_bytes','logical_file_and_link_bytes'),
                                     ('new_allocated_increments','allocated_bytes','allocated_bytes')]:
        last=old.get('last_'+key,baseline_totals[baseline_key])
        counters[channel]=old.get('channels',{}).get(channel,0)+max(0,snapshot['known_outputs'][key]-last)
    start_fs={x['device']:x['available_to_user_bytes']for x in baseline['filesystem_capacity']}
    last_fs=old.get('last_filesystem_available',start_fs)
    last_fs={int(k):v for k,v in last_fs.items()}
    current_fs={x['device']:x['available_bytes']for x in snapshot['filesystems']}
    counters['new_filesystem_increments']=old.get('channels',{}).get('new_filesystem_increments',0)+sum(
        max(0,last_fs.get(device,start_fs.get(device,available))-available)for device,available in current_fs.items())
    counters['shared_charge_delta']=max(0,snapshot['current_retained_budget_bytes']-baseline['current_retained_budget_bytes'])
    counters['deduplicated_actual_new_roots']=max(combined['logical_bytes'],combined['allocated_bytes'])
    charge=max(*counters.values(),old.get('charged_B',0))
    dt=max(1e-6,snapshot['monotonic_wall']-old.get('monotonic_wall',snapshot['monotonic_wall']))
    observed=max(0,charge-old.get('charged_B',charge))/dt
    rate=max(MIN_RATE,snapshot['predictive_growth_allowance_bytes_per_s'],observed)
    prediction=int(charge+rate*GRACE_S+ATOMIC_BYTES+CLEANUP_BYTES)
    run_bytes=max(run_usage['logical_bytes'],run_usage['allocated_bytes'])
    engineering_bytes=max(engineering['logical_bytes'],engineering['allocated_bytes'])
    return dict(schema='R80_original_conservative_round_prediction/v1',monotonic_wall=snapshot['monotonic_wall'],
        charged_B=charge,channels=counters,run_B=run_bytes,engineering_B=engineering_bytes,
        hard_B=ROUND_HARD_BYTES,run_soft_B=6_800_000_000,engineering_allowance_B=268_435_456,
        positive_growth_rate_B_s=observed,predictive_rate_B_s=rate,predictive_grace_s=GRACE_S,
        atomic_allowance_B=ATOMIC_BYTES,cleanup_reserve_B=CLEANUP_BYTES,
        projected_full_stop_B=prediction,projected_margin_B=ROUND_HARD_BYTES-prediction,
        predictive_soft_stop=prediction>=ROUND_HARD_BYTES or run_bytes>=6_800_000_000 or engineering_bytes>=268_435_456,
        hard_stop=charge>=ROUND_HARD_BYTES,soft_stop_requires_owner_handshake=True,no_refund=True,
        last_logical_bytes=snapshot['known_outputs']['logical_bytes'],last_allocated_bytes=snapshot['known_outputs']['allocated_bytes'],
        last_filesystem_available=current_fs)

def observe(run,profile,starting=False,count_files=True,previous=None):
    started=time.monotonic();policy=json.loads(POLICY.read_text());binding=json.loads((S/'BUDGET_BINDING.json').read_text())
    if (hashlib.sha256(POLICY.read_bytes()).hexdigest()!=binding['active_shared_policy_sha256'] or
        policy.get('hard_bytes')!=HARD_BYTES or policy.get('enforcement_metric')!='current_retained_budget_bytes' or
        policy.get('single_round_hard_bytes')!=ROUND_HARD_BYTES or
        policy.get('historical_counter_can_stop_new_experiment')is not False):
        raise RuntimeError('R80 current-retained budget source/authority differs')
    expected=dict(max_run_bytes=HARD_BYTES,minimum_start_available_bytes=107374182400,
                  minimum_runtime_available_bytes=107374182400)
    if profile['raw_storage_budget']!=expected:raise RuntimeError('Original filesystem/shared cap contract differs')
    d=json.loads(SESSION.read_text())
    if str(Path(run).resolve())not in d['roots']:raise RuntimeError('R80 run not registered')
    known=current()
    if known['scan_errors']:raise RuntimeError('Current-retained metadata census failed: '+str(known['scan_errors'][:3]))
    fs=[capacity('/home'),capacity('/tmp')]
    wall=time.monotonic();retained=known['current_retained_budget_bytes']
    dt=wall-previous['monotonic_wall']if previous else 1.
    rate=max(MIN_RATE,max(0,retained-previous['current_retained_budget_bytes'])/max(1e-6,dt)if previous else 0.)
    result=dict(schema='go2_R80_current_retained_storage_observation/v1',monotonic_wall=wall,
        current_retained_budget_bytes=retained,remaining_retained_budget_bytes=HARD_BYTES-retained,
        budget_basis='current_retained_experiment_data',enforcement_metric='current_retained_budget_bytes',
        conservative_session_growth_bytes=retained,
        legacy_field_alias_semantics='conservative_session_growth_bytes is current retention only, never historical cumulative growth',
        known_outputs=known,known_output_growth_bytes=retained,filesystems=fs,hard_bytes=HARD_BYTES,
        soft_stop_bytes=SOFT_BYTES,cleanup_reserve_bytes=CLEANUP_BYTES,predictive_growth_allowance_bytes_per_s=rate,
        single_atomic_write_allowance_bytes=ATOMIC_BYTES,size_check_interval_wall_s=1.,
        actual_census_elapsed_s=known['elapsed_s'],actual_observation_interval_s=dt if previous else None,
        original_full_census_parity_receipt=str(S/'RETAINED_OBSERVER_PARITY.json'),
        historical_counter_used_for_capacity=False,deletion_recovers_shared_capacity=True,no_deletion=True,
        session_file=str(SESSION),active_policy_file=str(POLICY),stop_reasons=[])
    basefile=S/'PRE_COPY_RETAINED_BASELINE.json'
    if hashlib.sha256(basefile.read_bytes()).hexdigest()!=binding['pre_copy_inventory_sha256']:
        raise RuntimeError('R80 original pre-copy retained baseline changed')
    baseline=json.loads(basefile.read_text());copy=json.loads((S/'COPY_RECEIPT.json').read_text());candidate=Path(copy['candidate'])
    round_row=round_evaluate(result,baseline,usage([run]),usage([S,candidate]),usage([S,candidate,run]),
        previous.get('R77_round_budget')if previous else None)
    result['R77_round_budget']=round_row  # Original runner key; semantics recorded above.
    result['R80_round_budget']=round_row
    reasons=result['stop_reasons']
    if retained>=SOFT_BYTES:reasons.append('current_retained_shared_soft_stop')
    if retained+rate*GRACE_S+ATOMIC_BYTES>=SOFT_BYTES:reasons.append('projected_current_retained_stop_grace')
    for item in fs:
        reserve=policy['filesystem_minimums'][item['path']]
        if item['available_bytes']<reserve:reasons.append('filesystem_reserve_'+item['path'])
    if round_row['hard_stop']:reasons.append('R80_hard_round_budget')
    result['whole_observe_elapsed_s']=time.monotonic()-started
    if reasons:raise StorageStop(result)
    if starting and round_row['predictive_soft_stop']:raise RuntimeError('R80 insufficient predicted round startup/closure budget')
    return result

class Watchdog:
    """Original one-second wait and owned-runner stop/normal-close interface."""
    def __init__(self,run,profile,stop_runner):
        self.run=Path(run);self.profile=profile;self.stop_runner=stop_runner
        self.done=threading.Event();self.thread=None;self.last=None;self.error=None
    def start(self):
        self.last=observe(self.run,self.profile,starting=True)
        self.thread=threading.Thread(target=self._loop,name='R80_retained_storage_watchdog',daemon=True)
        self.thread.start();return self
    def _loop(self):
        try:
            with(self.run/'storage_observations.jsonl').open('x',buffering=1)as stream:
                stream.write(json.dumps(self.last)+'\n')
                while not self.done.wait(1.):
                    row=observe(self.run,self.profile,previous=self.last);self.last=row
                    stream.write(json.dumps(row)+'\n')
        except Exception as error:
            self.error=str(error)
            row=error.observation if isinstance(error,StorageStop)else dict(error=str(error))
            row.update(schema='go2_R80_storage_stop/v1',monotonic_wall=time.monotonic(),
                stop_only_current_owned_runner=True,no_old_file_deleted=True)
            if not self.done.is_set():
                self.stop_runner()
                try:(self.run/'storage_stop.json').write_text(json.dumps(row,indent=2)+'\n')
                except Exception as receipt_error:self.error+='; stop receipt unavailable: '+str(receipt_error)
    def close(self):
        self.done.set()
        if self.thread:self.thread.join(timeout=5)
