#!/usr/bin/env python3
"""Independent production-CPP curve audit; inferred acceleration is not measured."""
import hashlib,json,pathlib,subprocess,tempfile
import numpy as np

ROOT=pathlib.Path(__file__).resolve().parents[3]
TEST=pathlib.Path(__file__).resolve().parent
CPP=ROOT/'scan_multifloor/ros2_ws/src/planner/traj_utils/src/polynomial_traj.cpp'
HEADER=CPP.parents[1]/'include'

def main():
    p=np.array([.304,-.152,-.00305]);e=np.array([-.05343189813310946,1.9996617509652172,-.001359314385753837])
    v=np.array([-.157,.048,.000442]);target=np.array([2.81,-.163,.00448]);target_v=np.array([.103,.0875,.000198])
    T=4*np.linalg.norm(e-p)/.3;candidates=[]
    for k in range(1,70):
        t=k*2.5/20/.3
        if t>=T:break
        s=t/T
        h=np.array([1-10*s**3+15*s**4-6*s**5,s-6*s**3+8*s**4-3*s**5,.5*s*s-1.5*s**3+1.5*s**4-.5*s**5,10*s**3-15*s**4+6*s**5])
        dh=np.array([-30*s*s+60*s**3-30*s**4,1-18*s*s+32*s**3-15*s**4,s-4.5*s*s+6*s**3-2.5*s**4,30*s*s-60*s**3+30*s**4])/T
        a=(target-p*h[0]-v*T*h[1]-e*h[3])/(T*T*h[2]);pred=p*dh[0]+v*T*dh[1]+a*T*T*dh[2]+e*dh[3]
        if np.linalg.norm(a)<=.35:candidates.append(dict(time=t,inferred_acceleration=a.tolist(),predicted_target_velocity=pred.tolist(),velocity_error=float(np.linalg.norm(pred-target_v))))
    candidates.sort(key=lambda d:d['velocity_error']);a=candidates[0]['inferred_acceleration']
    measured=[-.020989123379923983,-.004017430648470963,.0005771104838399873]
    cases=[('inferred_old_state',v,a,4,.3),('printed_v_zero_a',v,[0,0,0],4,.3),
           ('measured_zero_a',measured,[0,0,0],4,.3),('measured_physical_duration',measured,[0,0,0],2,.12),
           ('frozen_physical_duration',[0,0,0],[0,0,0],2,.12)]
    with tempfile.TemporaryDirectory(prefix='review_polynomial_') as directory:
        binary=pathlib.Path(directory)/'probe'
        subprocess.run(['g++','-O2','-std=c++17','-I/usr/include/eigen3','-I'+str(HEADER),str(TEST/'review_polynomial_probe.cpp'),str(CPP),'-o',str(binary)],check=True,capture_output=True)
        payload='\n'.join(' '.join(map(str,(name,*w,*acc,factor,speed))) for name,w,acc,factor,speed in cases)+'\n'
        output=subprocess.run([str(binary)],input=payload,text=True,capture_output=True,check=True).stdout
    rows={}
    for line in output.splitlines():
        name,*values=line.split();f=list(map(float,values))
        rows[name]=dict(duration=f[0],max_lateral_excursion=f[1],min_x=f[2],max_x=f[3],max_velocity=f[4],max_acceleration=f[5],length=f[6],local_time=f[7],local_target=f[8:11])
    checks={
        'inferred_old_state_matches_logged_east_target':np.linalg.norm(np.array(rows['inferred_old_state']['local_target'])-target)<1e-6,
        'inferred_velocity_matches_rounded_log':candidates[0]['velocity_error']<.001,
        'old_derivatives_can_exceed_physical_speed':rows['inferred_old_state']['max_velocity']>.3,
        'actual_measured_velocity_no_east_local_target':np.linalg.norm(np.array(rows['measured_zero_a']['local_target'])-e)<1e-6,
        'frozen_zero_derivatives_no_lateral_overshoot':rows['frozen_physical_duration']['max_lateral_excursion']<1e-8,
        'physical_duration_keeps_candidate_peak_velocity':rows['frozen_physical_duration']['max_velocity']<=.12 and rows['measured_physical_duration']['max_velocity']<=.12}
    result={'scope':'Production PolynomialTraj.cpp numerical regression; no occupancy/collision or full-physics acceptance. Old acceleration inferred from rounded logs, not directly recorded. Candidate frozen/measured states are controlled comparisons, not a claim that production state-selection logic was executed.',
            'checks':{k:bool(v) for k,v in checks.items()},'passed':bool(all(checks.values())),
            'inverse_fit':candidates[:5],'cases':rows,'source_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [CPP,TEST/'review_polynomial_probe.cpp']}}
    out=TEST.parent/'test_results/review_run11_polynomial_regression.json';out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'checks':result['checks'],'report':str(out)},indent=2))
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
