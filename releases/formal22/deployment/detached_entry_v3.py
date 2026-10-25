"""Detach only the existing bounded execute/sampler so transport loss cannot kill receipts."""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--operation',choices=('execute','sample'),required=True)
    a=p.parse_args();trial=a.trial.resolve();root=Path(__file__).resolve().parents[1]
    if trial.parent!=root/'trials':raise ValueError('Existing trial under this experiment required')
    authority=json.loads((trial/'AUTHORITY.json').read_text())
    if dt.datetime.now(dt.timezone.utc)>=dt.datetime.fromisoformat(authority['deadline_utc']):
        raise ValueError('Original concrete deadline expired')
    if Path('/proc/1/comm').read_text().strip()!='systemd':raise ValueError('Host namespace required')
    if a.operation=='execute':
        helper=root/'deployment/round27_v3.py'
        cmd=[sys.executable,'-B',str(helper),'execute','--trial',str(trial)]
        if (trial/'EXECUTION_INTENT.json').exists():raise ValueError('Existing execute intent; inspect, never relaunch')
    else:
        helper=root/'protocol_reduction/sample_controller_threads.py'
        cmd=[sys.executable,'-B',str(helper),'--trial',str(trial)]
        if not (trial/'EXECUTION_INTENT.json').exists():raise ValueError('Execute intent required before sampler')
    receipt=trial/('DETACHED_'+a.operation.upper()+'_IDENTITY.json')
    with receipt.open('x')as record:
        with (trial/('DETACHED_'+a.operation.upper()+'_OUTPUT.log')).open('x')as log:
            child=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True,close_fds=True,cwd=str(root))
        raw=(Path('/proc')/str(child.pid)/'stat').read_text();f=raw[raw.rfind(')')+2:].split()
        value=dict(schema='existing_bounded_entry_detached/v1',operation=a.operation,pid=child.pid,
            start_ticks=int(f[19]),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            argv=cmd,UTC=dt.datetime.now(dt.timezone.utc).isoformat(),deadline_utc=authority['deadline_utc'],
            authority_or_runtime_changed=False,signals_sent=False)
        json.dump(value,record,indent=2);record.write('\n')
    print(json.dumps(value),flush=True)


if __name__=='__main__':main()
