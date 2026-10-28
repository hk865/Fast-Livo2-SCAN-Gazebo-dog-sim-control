"""Low-cost component check with synthetic native frames; never navigation PASS."""
from pathlib import Path
import argparse,json,os,socket,subprocess,sys,time,uuid
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from common.config import path

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',type=Path,required=True);a=ap.parse_args()
    run=a.run.resolve();address='/tmp/actor_component_'+uuid.uuid4().hex[:10]+'.sock'
    env={**os.environ,'PYTHONPATH':str(ROOT/'src')+':'+os.environ.get('PYTHONPATH',''),'CUDA_VISIBLE_DEVICES':'','NAVIGATION_RECORD_RAW':'0'}
    log=(run/'actor_component.log').open('x')
    proc=subprocess.Popen([str(path('actor_python')),'-B','-m','control.policy.worker','--run',str(run),'--socket',address],env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+30
        while not(run/'worker_ready').exists():
            if proc.poll()is not None:raise RuntimeError('Actor initialization failed; see component log')
            if time.monotonic()>deadline:raise TimeoutError('Actor readiness')
            time.sleep(.05)
        with socket.socket(socket.AF_UNIX) as conn:
            conn.settimeout(3);conn.connect(address)
            for index in range(8):
                s=np.zeros(64,dtype='<f8');s[0]=.02*(index+1);s[3]=.30;s[4]=1
                s[14:26]=[-.1,.8,-1.5,.1,.8,-1.5,-.1,1.,-1.5,.1,1.,-1.5]
                s[51:55]=1;conn.sendall(s.tobytes());data=b''
                while len(data)<128:
                    chunk=conn.recv(128-len(data))
                    if not chunk:raise EOFError('Actor closed before full response')
                    data+=chunk
                response=np.frombuffer(data,dtype='<f8')
                if not np.isfinite(response).all() or response[12]!=0:raise ValueError('Nonfinite/fault response')
        proc.wait(timeout=8)
        rows=json.loads((run/'closed_loop_executor_ack.json').read_text())['entries']
        if len(rows)!=7 or any(x['ack_native_clock_ns']<=x['stamp_ns'] for x in rows):raise ValueError('ACK did not follow the next native frame')
        result=dict(component='Student socket/frame/next-request ACK',passed=True,
            synthetic_native_frames=8,responses=8,causal_acks=7,navigation_pass_claimed=False,
            actor_exit_code=proc.returncode)
        (run/'actor_component_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
        return 0
    finally:
        if proc.poll()is None:proc.terminate();proc.wait(timeout=8)
        log.close()
        try:Path(address).unlink()
        except FileNotFoundError:pass
if __name__=='__main__':raise SystemExit(main())
