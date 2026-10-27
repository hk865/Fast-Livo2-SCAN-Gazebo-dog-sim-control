"""Current storage capacity and original conservative growth/stop arithmetic."""
from pathlib import Path
import datetime as dt
import json,os,threading,time
from storage_usage import usage
HERE=Path(__file__).resolve().parent
HARD_BYTES=170000000000
SOFT_BYTES=HARD_BYTES-10000000000
ROUND_HARD_BYTES=10000000000
MIN_RATE=17*1024**2
GRACE_S=122.
ATOMIC_BYTES=134217728
CLEANUP_BYTES=536870912
def capacity(path):
    p=Path(path);st=os.statvfs(p)
    return dict(path=str(p),device=os.stat(p).st_dev,available_bytes=st.f_bavail*st.f_frsize)
def register(run):
    from project_config import ROOT
    run=Path(run).resolve()
    if not run.is_dir()or run.stat().st_uid!=os.getuid()or not run.is_relative_to(ROOT/'runs'):raise RuntimeError('Owned project run required')
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
    from runtime_guard import census,remaining_seconds,work_deadline_withdrawn
    from runtime_limits import configuration as resource_configuration
    renewed=resource_configuration(profile)
    hard=renewed['maximum_retained_bytes'] if renewed is not None else 130000000000
    soft=renewed['soft_stop_bytes'] if renewed is not None else SOFT_BYTES
    run=Path(run).resolve();known=census()
    if known['scan_errors']:raise RuntimeError('Current retained census failed')
    expected=dict(max_run_bytes=hard,minimum_start_available_bytes=107374182400,minimum_runtime_available_bytes=107374182400)
    if profile['raw_storage_budget']!=expected:raise RuntimeError('Original storage reserve profile changed')
    fs=[capacity('/home'),capacity('/tmp')]
    baseline_path=run/'r85_round_storage_baseline.json'
    if not baseline_path.exists():
        if not starting:raise RuntimeError('Fresh round baseline missing')
        row=dict(schema='r85_current_retained_round_baseline/v1',UTC=dt.datetime.now(dt.timezone.utc).isoformat(),
            current_retained_budget_bytes=known['current_retained_budget_bytes'],
            totals=dict(allocated_bytes=known['allocated_bytes'],logical_file_and_link_bytes=known['logical_bytes']),
            filesystem_capacity=[dict(path=x['path'],device=x['device'],available_to_user_bytes=x['available_bytes'])for x in fs],
            previous_round_evidence_retained=True,no_previous_round_reclassification=True)
        with baseline_path.open('x')as f:json.dump(row,f,indent=2);f.write('\n')
    baseline=json.loads(baseline_path.read_text());wall=time.monotonic()
    elapsed=wall-previous['monotonic_wall']if previous else 1.
    retained=known['current_retained_budget_bytes']
    rate=max(MIN_RATE,max(0,retained-previous['current_retained_budget_bytes'])/max(elapsed,1e-6)if previous else 0.)
    result=dict(schema='r85_current_retained_and_round_storage/v1',monotonic_wall=wall,
        known_outputs=known,current_retained_budget_bytes=retained,remaining_retained_budget_bytes=hard-retained,
        conservative_session_growth_bytes=retained,filesystems=fs,hard_bytes=hard,
        soft_stop_bytes=soft,predictive_growth_allowance_bytes_per_s=rate,
        historical_counter_used_for_capacity=False,budget_basis='current_retained_experiment_data',
        round_baseline=str(baseline_path),stop_reasons=[],no_deletion=True)
    if renewed is not None:
        result['resource_renewal']=renewed
        result['resource_limits_are_numeric_only']=True
    engineering=usage([HERE])
    combined=usage([HERE,run])
    round_row=round_evaluate(result,baseline,usage([run]),engineering,combined,
        previous.get('R77_round_budget')if previous else None)
    result['R77_round_budget']=round_row;result['R85_round_budget']=round_row
    reasons=result['stop_reasons']
    if retained+rate*GRACE_S+ATOMIC_BYTES>=soft:reasons.append('projected_retained_capacity_stop')
    if round_row['predictive_soft_stop']:reasons.append('fresh_round_predictive_stop')
    if round_row['hard_stop']:reasons.append('fresh_round_hard_stop')
    for x in fs:
        if x['available_bytes']<{'/home': 107374182400, '/tmp': 85899345920}[x['path']]:reasons.append('filesystem_reserve_'+x['path'])
    # Leave owned drain time before the explicit hard deadline.
    if not work_deadline_withdrawn(profile) and remaining_seconds(profile)<=GRACE_S:reasons.append('overnight_deadline_drain')
    if reasons:raise StorageStop(result)
    return result

class Watchdog:
    def __init__(self,run,profile,stop_runner):
        self.run=Path(run);self.profile=profile;self.stop_runner=stop_runner
        self.done=threading.Event();self.last=None;self.error=None;self.thread=None
    def start(self):
        self.last=observe(self.run,self.profile,starting=True)
        self.thread=threading.Thread(target=self._loop,daemon=True);self.thread.start();return self
    def _loop(self):
        try:
            with (self.run/'r85_storage_observations.jsonl').open('x',buffering=1)as stream:
                stream.write(json.dumps(self.last)+'\n')
                while not self.done.wait(1.):
                    self.last=observe(self.run,self.profile,previous=self.last)
                    stream.write(json.dumps(self.last)+'\n')
        except Exception as e:
            self.error=type(e).__name__+': '+str(e)
            row=e.observation if isinstance(e,StorageStop)else dict(error=self.error)
            row.update(schema='r85_storage_stop/v1',monotonic_wall=time.monotonic(),
                stop_only_current_owned_runner=True,no_old_file_deleted=True)
            with (self.run/'r85_storage_stop.json').open('x')as out:json.dump(row,out,indent=2);out.write('\n')
            self.stop_runner()
    def close(self):
        self.done.set()
        if self.thread:self.thread.join(timeout=3.)
        return self.last
