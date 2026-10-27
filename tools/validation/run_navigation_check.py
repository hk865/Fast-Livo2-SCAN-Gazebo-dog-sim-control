"""Explicit development test around the unchanged daily entry."""
from pathlib import Path
import argparse,datetime as dt,json,os,signal,subprocess,sys,threading,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from daily_runtime import environment

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--label',default='daily_validation');args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    log=(output/'entry.log').open('x');process=subprocess.Popen(['/usr/bin/python3','-B',str(ROOT/'start.py'),'--domain','231','--label',args.label,'--resource-check','--duration','850'],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
    run=None;observer=None
    try:
        for line in process.stdout:
            log.write(line);log.flush();print(line,end='',flush=True)
            try:message=json.loads(line)
            except ValueError:continue
            if message.get('stage')=='prepared':run=Path(message['directory']);break
        if run is None:raise RuntimeError('Daily entry did not prepare')
        (output/'run.json').write_text(json.dumps(dict(run=str(run),launcher_pid=process.pid,started_UTC=dt.datetime.now(dt.timezone.utc).isoformat()),indent=2)+'\n')
        def drain():
            for line in process.stdout:log.write(line);log.flush();print(line,end='',flush=True)
        thread=threading.Thread(target=drain,daemon=True);thread.start()
        profile=json.loads((run/'runtime_context.json').read_text())['profile'];env=environment(run,231,profile,'/tmp/unused_observer_socket')
        observer=subprocess.Popen(['/usr/bin/python3','-B',str(ROOT/'tools/validation/navigation_observer.py'),'--run',str(run),'--output',str(output/'observations')],cwd=ROOT,env=env)
        result=observer.wait();return result
    finally:
        if process.poll()is None:
            os.kill(process.pid,signal.SIGINT)
            try:process.wait(timeout=35)
            except subprocess.TimeoutExpired:raise RuntimeError('Daily entry did not finish owned cleanup')
        if observer is not None and observer.poll()is None:observer.terminate();observer.wait(timeout=5)
        if run is not None:(output/'entry_exit.json').write_text(json.dumps(dict(run=str(run),exit_code=process.returncode,finished_UTC=dt.datetime.now(dt.timezone.utc).isoformat()),indent=2)+'\n')
        log.close()

if __name__=='__main__':raise SystemExit(main())
