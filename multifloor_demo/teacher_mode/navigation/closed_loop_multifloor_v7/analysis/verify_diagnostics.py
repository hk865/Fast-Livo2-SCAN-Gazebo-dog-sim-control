#!/usr/bin/env python3
"""Read-only binary/index/schema and recorded IMU algebra verification.

No ROS, simulator, state correction or process control. An integrity pass means
the diagnostic records are structurally complete and internally consistent;
it is never a localization, motion or navigation acceptance result.
"""
from __future__ import annotations
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import numpy as np

MAGIC=b'FLIVODIAG0001LE\0'
HEADER=struct.Struct('<7Q')
INDEX_FIELDS=['offset','kind','sequence','stamp_ns','stage','iteration','level','n_values']
MAX_VALUES=32*1024*1024//8


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as stream:
        for chunk in iter(lambda:stream.read(1<<20),b''):h.update(chunk)
    return h.hexdigest()


def signed(value):
    return value if value<1<<63 else value-(1<<64)


def integer(value,label):
    if not np.isfinite(value)or value<0 or value!=int(value):raise ValueError('Invalid '+label)
    return int(value)


def schema_files(run=None):
    here=Path(__file__).resolve().parents[1]
    names=('caller_diagnostic_schema.json','lio_diagnostic_schema.json','vio_diagnostic_schema.json')
    if run is None:return [here/name for name in names]
    snapshots=json.loads((run/'navigation_source_snapshots.json').read_text())
    files=[]
    for name in names:
        rows=[row for source,row in snapshots.items()if Path(source).name==name]
        if len(rows)!=1:raise ValueError('No unique frozen schema '+name)
        path=Path(rows[0]['snapshot']).resolve()
        if not path.is_relative_to((run/'sources').resolve())or sha(path)!=rows[0]['sha256']:
            raise ValueError('Frozen schema hash/path differs '+name)
        files.append(path)
    return files


def load_schemas(files):
    specs={};provenance={}
    for path in files:
        document=json.loads(Path(path).read_text())
        for kind,spec in document['kinds'].items():
            if int(kind)in specs:raise ValueError('Duplicate schema kind '+kind)
            specs[int(kind)]=spec
        provenance[str(path)]={'sha256':sha(path),'schema':document['schema']}
    return specs,provenance


def validate_length(kind,values,specs):
    n=len(values);spec=specs.get(kind)
    if kind==4:  # Actual post-preprocess input cloud: n, width=5, head_s.
        if n<3:raise ValueError('kind4 short prefix')
        rows=integer(values[0],'kind4 rows');width=integer(values[1],'kind4 width')
        if width!=5 or n!=3+rows*width:raise ValueError('kind4 row layout differs')
        return
    if spec is None:raise ValueError('Unknown record kind '+str(kind))
    if 'length'in spec:
        if n!=spec['length']:raise ValueError(f'kind{kind} length {n} != {spec["length"]}')
    elif 'payload_width'in spec:
        if n!=spec['payload_width']:raise ValueError('kind'+str(kind)+' aggregate width differs')
    elif kind==14:
        if n<27:raise ValueError('kind14 short prefix/state')
        rows=integer(values[0],'kind14 rows');width=integer(values[1],'kind14 width')
        if width!=15 or n!=27+rows*width:raise ValueError('kind14 row layout differs')
    elif kind in (101,102,103):
        prefix=len(spec['prefix']);count_offset=2 if kind==103 else 1
        if n<prefix:raise ValueError('kind'+str(kind)+' short prefix')
        rows=integer(values[count_offset],'rows');width=integer(values[count_offset+1],'width')
        if width!=spec['row_width']or n!=prefix+rows*width:raise ValueError('kind'+str(kind)+' row layout differs')
    elif kind==200:
        if n<1:raise ValueError('kind200 missing phase')
        phase=str(integer(values[0],'kind200 phase'));phase_spec=spec['phases'].get(phase)
        if phase_spec is None or n!=phase_spec['length']:raise ValueError('kind200 phase/length differs')
    elif kind==201:
        if n==spec['skipped_length']:return
        if n<12:raise ValueError('kind201 short metadata')
        columns=integer(values[1],'kind201 H columns')
        if columns not in (6,7)or n!=828+columns*columns+columns:
            raise ValueError('kind201 iteration width differs')
    elif kind==203:
        if n<5:raise ValueError('kind203 short metadata')
        rows=integer(values[1],'kind203 rows');columns=integer(values[2],'kind203 columns')
        if columns not in (6,7)or n!=5+rows*columns+rows:raise ValueError('kind203 pixel width differs')
    else:raise ValueError('No width validator for record kind '+str(kind))


def so3_exp(vector):
    vector=np.asarray(vector,float);angle=np.linalg.norm(vector)
    skew=np.array([[0,-vector[2],vector[1]],[vector[2],0,-vector[0]],[-vector[1],vector[0],0]])
    if angle<1e-8:return np.eye(3)+skew+.5*(skew@skew)
    return np.eye(3)+math.sin(angle)/angle*skew+(1-math.cos(angle))/angle**2*(skew@skew)


def imu_algebra(values):
    if len(values)!=58 or not np.isfinite(values).all():raise ValueError('kind20 needs58 finite values')
    dt=values[4]
    if dt<0:raise ValueError('Negative actual integration dt')
    gyro,accel,gravity=values[10:13],values[13:16],values[22:25]
    v0,p0,r0=values[25:28],values[28:31],values[31:40].reshape(3,3)
    world,v1,p1,r1=values[40:43],values[43:46],values[46:49],values[49:58].reshape(3,3)
    return {
        'world_acceleration':float(np.max(np.abs(world-(r1@accel+gravity)))),
        'velocity':float(np.max(np.abs(v1-(v0+world*dt)))),
        'position':float(np.max(np.abs(p1-(p0+v0*dt+.5*world*dt*dt)))),
        'rotation':float(np.max(np.abs(r1-r0@so3_exp(gyro*dt)))),
        'rotation_orthogonality':float(np.max(np.abs(r1.T@r1-np.eye(3)))),
    }


def verify(directory,specs,provenance,required=(1,10,11,12,13,20)):
    directory=Path(directory);binary=directory/'records.bin';index=directory/'index.csv'
    counts=Counter();errors=[];nonfinite=Counter();max_error=Counter();contexts={};offset=16
    detail_counts=Counter();imu_count=0;record_count=0
    def error(message):
        if len(errors)<100:errors.append(message)
    with binary.open('rb')as data,index.open(newline='')as csv_file:
        if data.read(16)!=MAGIC:raise ValueError('Binary magic/endian/version differs')
        reader=csv.DictReader(csv_file)
        if reader.fieldnames!=INDEX_FIELDS:raise ValueError('Index header differs')
        while True:
            raw=data.read(HEADER.size)
            if not raw:break
            if len(raw)!=HEADER.size:raise ValueError('Truncated binary header at'+str(offset))
            head=HEADER.unpack(raw);kind,seq,stamp,stage,iteration,level,n=head
            if n>MAX_VALUES:raise ValueError('Payload exceeds bounded logger record size')
            payload=data.read(n*8)
            if len(payload)!=n*8:raise ValueError('Truncated binary payload at'+str(offset))
            row=next(reader,None)
            if row is None:raise ValueError('Missing index row at'+str(offset))
            if any(row.get(name)is None for name in INDEX_FIELDS):raise ValueError('Incomplete index row')
            expected=(offset,*head)
            if tuple(int(row[name])for name in INDEX_FIELDS)!=expected:raise ValueError('Index/header mismatch at'+str(offset))
            values=np.frombuffer(payload,dtype='<f8');counts[kind]+=1;record_count+=1
            validate_length(kind,values,specs)
            nonfinite[kind]+=int(np.count_nonzero(~np.isfinite(values)))
            if 115_000_000_000<=stamp<=165_000_000_000:detail_counts[kind]+=1
            if kind in (1,2,3,4):
                if seq!=0 or stage!=0:error(f'raw kind{kind} has nonraw context at{offset}')
            elif seq==0 or stage not in (1,2,3):error(f'estimator kind{kind} missing context at{offset}')
            if kind==10:
                if seq in contexts:error('Repeated stage sequence'+str(seq))
                contexts[seq]=(stamp,stage)
                target=values[4]if stage==2 else values[5]
                if abs(stamp-target*1e9)>1:error('kind10 phase timestamp differs at'+str(offset))
            elif kind not in (1,2,3,4)and contexts.get(seq)!=(stamp,stage):
                error('Record does not match preceding kind10 context at'+str(offset))
            if kind in (1,2,3,4,10,11,12,13,14,20)and not np.isfinite(values).all():
                error(f'Unexpected nonfinite caller/IMU values kind{kind} at{offset}')
            if kind==20:
                try:
                    residuals=imu_algebra(values);imu_count+=1
                    for name,value in residuals.items():
                        max_error[name]=max(max_error[name],value)
                        tolerance=2e-6 if name.startswith('rotation')else 2e-8
                        if value>tolerance:error(f'kind20 {name} mismatch {value} at{offset}')
                except ValueError as exc:error(str(exc)+' at'+str(offset))
            offset+=HEADER.size+n*8
        if next(reader,None)is not None:raise ValueError('Extra index rows after binary end')
    stats=json.loads((directory/'writer_stats.json').read_text())
    if stats.get('final')is not True:error('Writer did not confirm final drain')
    if stats.get('writer_io_failed')is not False:error('Writer I/O failed/unavailable')
    if stats.get('dropped')!=0:error('Diagnostic queue dropped records')
    if stats.get('written')!=record_count:error('Writer count differs from binary/index')
    if stats.get('attempted')!=record_count:error('Attempted count differs from complete archive')
    if stats.get('file_bytes')!=offset or binary.stat().st_size!=offset:error('Final file size differs')
    for kind in required:
        if counts[kind]==0:error('Missing required caller/IMU kind'+str(kind))
    return {'schema':'teacher_v7_diagnostic_integrity/v1','directory':str(directory.resolve()),
        'status':'failed'if errors else'passed','meaning':'Diagnostic integrity only; localization/motion/navigation remain independently evaluated',
        'record_count':record_count,'counts':dict(counts),'detail_window_counts':dict(detail_counts),
        'nonfinite_values_by_kind':dict(nonfinite),'imu_algebra_checked_pairs':imu_count,
        'imu_algebra_max_absolute_errors':dict(max_error),'first100_errors':errors,
        'writer_stats':stats,'schema_provenance':provenance,
        'reader_sha256':sha(__file__),'binary_sha256':sha(binary),'index_sha256':sha(index)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    location=parser.add_mutually_exclusive_group(required=True)
    location.add_argument('--run',type=Path);location.add_argument('--directory',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--fixture',action='store_true',help='Check structure without requiring production caller kinds')
    args=parser.parse_args();run=args.run.resolve()if args.run else None
    directory=run/'fastlivo_diagnostics'if run else args.directory.resolve()
    try:
        specs,provenance=load_schemas(schema_files(run))
        report=verify(directory,specs,provenance,required=()if args.fixture else(1,10,11,12,13,20))
    except (OSError,ValueError,KeyError,TypeError)as exc:
        report={'schema':'teacher_v7_diagnostic_integrity/v1','status':'failed','directory':str(directory),
                'error':f'{type(exc).__name__}: {exc}','meaning':'No diagnostic integrity confirmation'}
    encoded=json.dumps(report,indent=2,allow_nan=False)+'\n'
    if args.output:
        with args.output.open('x')as stream:stream.write(encoded)
    print(encoded,end='');return 0 if report['status']=='passed'else 1


if __name__=='__main__':sys.exit(main())
