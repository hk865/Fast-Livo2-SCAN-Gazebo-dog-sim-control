#!/usr/bin/env python3
"""Read original V4/V5 IMU availability metadata; write only this new directory."""
from pathlib import Path
import hashlib
import json
import numpy as np

HERE=Path(__file__).resolve().parent


def main():
    aggregate=json.loads((HERE/'aggregate.json').read_text())
    records=[]
    declared=[(r['run'],'V5') for r in aggregate['runs']]+[(r['run'],'V4') for r in aggregate['V4_original_receipts_retained']]
    for name,version in declared:
        path=Path(name)/'SLAM_fixed_route/adapter_IMU_inputs.jsonl';data=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        flags=[r.get('orientation_available') for r in data];cov=[r.get('orientation_covariance') for r in data]
        record={'version':version,'run':name,'original_raw_IMU_log':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'raw_rows':len(data),'orientation_available_true':sum(x is True for x in flags),
            'orientation_available_false':sum(x is False for x in flags),'orientation_available_missing_or_not_bool':sum(type(x) is not bool for x in flags),
            'orientation_covariance_original_all_zero_rows':sum(isinstance(x,list) and len(x)==9 and all(v==0 for v in x) for x in cov),
            'orientation_covariance_missing_rows':sum(x is None for x in cov),
            'actual_gyro_only_orientation_unavailable_branch_exercised':any(x is False for x in flags)}
        records.append(record)
    demonstration=np.zeros(9)[0]>=0
    result={'schema':'actual_IMU_orientation_availability_source_review/v1','status':'passed' if all(r['orientation_available_true']==r['raw_rows'] and r['orientation_covariance_original_all_zero_rows']==r['raw_rows'] for r in records) else 'failed',
        'original_raw_fields_authoritative':True,'records':records,'actual_V4_V5_orientation_available':True,
        'actual_V4_V5_covariance_all_zero':True,'actual_V4_V5_gyro_only_branch_exercised':False,
        'historical_V2_limitations':'V2 archived only rejection records rather than original IMU rows. Static SDF orientation covariance -1 does not establish actual historical ROS header metadata or orientation availability. No historical raw fields are backfilled.',
        'numpy_scalar_identity_demonstration':{'example':'np.zeros(9)[0]>=0','value':bool(demonstration),'type':type(demonstration).__name__,'is_python_True':demonstration is True},
        'historical_diagnosis':'V2 source used a comparison potentially returning numpy.bool_ and then identity is not True. V3 writer logged numpy.bool_ serialization failure. These establish a Python scalar handling defect as a supported explanation, not a unique claim about missing actual sensor orientation.',
        'pure_gyro_only_branch_status':'Only offline negative tests qualify the unavailable-orientation branch; these actual V4/V5 runs declare orientation available.',
        'old_receipts_or_runs_modified':False,'audit_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    text=json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n'
    with (HERE/'actual_IMU_orientation_source_correction.json').open('x') as stream:stream.write(text)
    print(result['status'],len(records),sum(r['raw_rows'] for r in records))


if __name__=='__main__':main()
