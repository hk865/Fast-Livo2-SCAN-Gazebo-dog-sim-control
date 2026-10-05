#!/usr/bin/env python3
"""Actual runner control flow, mock files/children only; no ROS/Gazebo/model."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

TEACHER=Path(__file__).resolve().parents[2]
SCRIPT=TEACHER/'scripts/run_test.py'


def probe(signum):
    spec=importlib.util.spec_from_file_location('mock_runner_'+str(signum),SCRIPT)
    module=importlib.util.module_from_spec(spec);sys.path.insert(0,str(SCRIPT.parent));spec.loader.exec_module(module)
    original_popen=subprocess.Popen;original_run=subprocess.run;saved={s:signal.getsignal(s)for s in(signal.SIGTERM,signal.SIGINT)}
    oldargv=sys.argv[:];children=[];timers=[];exit_code=None;sentinel=None
    with tempfile.TemporaryDirectory(prefix='teacher-runner-owned-probe-')as directory:
        root=Path(directory);(root/'policy').mkdir();(root/'simulation/build').mkdir(parents=True)
        (root/'simulation/build/libteacher_actuator.so').write_bytes(b'MOCK ONLY; NEVER LOADED')
        (root/'simulation/prepare.py').write_text('# MOCK ONLY')
        child_code="""import signal,sys,time
from pathlib import Path
def done(s,f):
    Path(sys.argv[1]).write_text(str(s))
    time.sleep(.35)
    sys.exit(0)
signal.signal(signal.SIGINT,done)
signal.signal(signal.SIGTERM,done)
if len(sys.argv)>2:Path(sys.argv[2]).write_text('mock ready')
while True:time.sleep(.05)
"""
        (root/'policy/worker.py').write_text(child_code)
        sentinel=original_popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True)
        module.ROOT=root;module.PYTHON=Path(sys.executable)
        module.resource=lambda:{'processes':'controlled mock only','gpu':'900 MiB, 0 %'}
        def mock_run(cmd,*a,**kw):
            if cmd[0]=='ps':return subprocess.CompletedProcess(cmd,0,stdout='mock owned process list')
            if len(cmd)>1 and str(cmd[1])==str(root/'simulation/prepare.py'):
                run=Path(cmd[cmd.index('--output')+1]);(run/'world.sdf').write_text('<sdf><world name="MOCK"/></sdf>')
                return subprocess.CompletedProcess(cmd,0)
            return original_run(cmd,*a,**kw)
        def mock_popen(cmd,*a,**kw):
            if str(cmd[0])=='gz':
                run=Path(cmd[-1]).parent
                proc=original_popen([sys.executable,'-c',child_code,str(run/'mock_gazebo_signal.txt')],*a,**kw)
                children.append(proc)
                # First signal exercises InterruptedError. The repeated other
                # signal arrives while the mock child delays orderly cleanup.
                first=threading.Timer(.25,lambda:os.kill(os.getpid(),signum))
                second=threading.Timer(.35,lambda:os.kill(os.getpid(),signal.SIGINT if signum==signal.SIGTERM else signal.SIGTERM))
                first.start();second.start();timers.extend([first,second]);return proc
            if len(cmd)>1 and str(cmd[1])==str(root/'policy/worker.py'):
                run=Path(cmd[cmd.index('--run')+1])
                proc=original_popen([sys.executable,'-c',child_code,str(run/'mock_worker_signal.txt'),str(run/'worker_ready')],*a,**kw)
                children.append(proc);return proc
            raise AssertionError('Forbidden non-mock Popen: '+str(cmd))
        def evaluate_mock(run):
            manifest=json.loads((run/'runtime_manifest.json').read_text())
            # Full physical evaluator is intentionally not claimed: this
            # exercise only checks that the runner passes interruption to it.
            summary={'tests':[{'name':'mock_stand','status':'failed'if manifest['error']else'passed'}],
                     'runtime_error_received':manifest['error'],'scope':'MOCK lifecycle only'}
            (run/'summary.json').write_text(json.dumps(summary));return summary
        module.evaluate=evaluate_mock;subprocess.Popen=mock_popen;subprocess.run=mock_run
        sys.argv=[str(SCRIPT),'stand']
        try:
            try:module.main()
            except SystemExit as error:exit_code=error.code
            for timer in timers:timer.join(timeout=2.)
            run=next((root/'runs').glob('*_stand_r1_*'))
            manifest=json.loads((run/'runtime_manifest.json').read_text());summary=json.loads((run/'summary.json').read_text())
            receipt={'termination_signal':int(signum),'exit_code':exit_code,
                'expected_exit_code':128+int(signum),'runtime_error':manifest['error'],
                'owned_children':[{'pid':proc.pid,'returncode':proc.poll()}for proc in children],
                'unowned_sentinel_alive_after_runner_cleanup':sentinel.poll()is None,
                'failed_receipt_written':summary['tests'][0]['status']=='failed',
                'repeated_signal_during_cleanup_did_not_abort_receipt':(run/'resources_after.json').is_file(),
                'owned_signal_records':{p.name:p.read_text()for p in run.glob('mock_*_signal.txt')},
                'starts_ros':False,'starts_gazebo':False,'loads_teacher':False}
            assert exit_code==128+int(signum)
            assert all(proc.poll()==0 for proc in children)
            assert sentinel.poll()is None and receipt['failed_receipt_written']
            assert len(receipt['owned_signal_records'])==2
            return receipt
        finally:
            subprocess.Popen=original_popen;subprocess.run=original_run;sys.argv=oldargv
            for timer in timers:timer.cancel()
            for proc in children+[sentinel]:
                if proc is not None and proc.poll()is None:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2.)
            for s,handler in saved.items():signal.signal(s,handler)


def main():
    result={'schema':1,'status':'passed','runner_sha256':hashlib.sha256(SCRIPT.read_bytes()).hexdigest(),
        'probe_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'tests':[probe(signal.SIGTERM),probe(signal.SIGINT)],
        'physical_validation':'unverified; these were only Python mock children and mock files',
        'limitations':['Prepare/source-copy stage is outside runtime try; no runtime children yet, but no guaranteed receipt there',
            'Existing final wait/resource/evaluator exceptions can still interrupt receipt writing; not changed by this signal patch',
            'Popen-to-variable-assignment asynchronous signal race is not covered by this deterministic probe']}
    output=Path(__file__).with_name('receipt.json');output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'status':result['status'],'receipt':str(output),'tests':len(result['tests'])}))


if __name__=='__main__':main()
