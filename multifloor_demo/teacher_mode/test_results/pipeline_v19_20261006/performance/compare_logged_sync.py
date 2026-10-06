#!/usr/bin/env python3
"""Observe original DEMO_SYNC target coverage; no loss or causality PASS gate."""
import argparse
import json
import re
from pathlib import Path
from read_stages import sha


def compare(runs):
    value={'schema':'V19_logged_sync_completeness_descriptive/v1','rows':[],
        'interpretation':'Original DEMO_SYNC log statement only. complete0 at a1ns floating threshold is separated from missing physical IMU interval. Not fullbody replay or proof of sole cause.'}
    for run in runs:
        run=Path(run);path=run/'mapping.log';rows=[]
        for line in path.open():
            if '[DEMO_SYNC]'in line:
                m=re.search(r'camera=([\d.]+).*imu_newest=([\d.]+).*complete=(\d)',line)
                if not m:raise ValueError('Truncated DEMO_SYNC line')
                rows.append((float(m[1]),float(m[2]),int(m[3])))
        if not rows:raise ValueError('Missing DEMO_SYNC actual log')
        if len({x[0]for x in rows})!=len(rows):raise ValueError('Duplicate camera sync statement')
        lacks=[(a-b)*1e3 for a,b,c in rows if not c]
        value['rows'].append({'label':run.name,'sync_rows':len(rows),'complete0':len(lacks),
            'camera_exceeds_latest_IMU_more_than_1microsecond':sum(x>.001 for x in lacks),
            'max_missing_interval_ms':max(lacks,default=0),'mean_missing_interval_ms':sum(lacks)/len(lacks)if lacks else None,
            'source_log_sha256':sha(path)})
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,nargs='+',required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    with args.out.open('x')as f:json.dump(compare(args.run),f,indent=2);f.write('\n')


if __name__=='__main__':main()
