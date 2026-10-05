#!/usr/bin/env python3
"""Export exact logged LIO row inputs; never modify the sealed source run."""
from __future__ import annotations
import argparse, csv, datetime, hashlib, json, struct
from pathlib import Path
import numpy as np

HEADER=struct.Struct('<7Q')
MAGIC=b'FLIVODIAG0001LE\0'
HERE=Path(__file__).resolve().parent
DEFAULT_RUN=Path('/var/tmp/go2_teacher_simulation_20261005/20261005_205309_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_timing_r1_0b8d')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def key(row): return tuple(int(row[n]) for n in ('sequence','stamp_ns','stage','iteration','level'))
def read_record(f,row):
    f.seek(int(row['offset'])); h=f.read(HEADER.size)
    values=HEADER.unpack(h)
    # Index negative iteration/level fields are signed, header fields are uint64.
    expected=tuple(int(row[n])%(1<<64) for n in ('kind','sequence','stamp_ns','stage','iteration','level','n_values'))
    if values!=expected: raise ValueError('Index/header mismatch')
    p=f.read(values[-1]*8)
    if len(p)!=values[-1]*8: raise ValueError('Truncated record')
    return np.frombuffer(p,dtype='<f8').copy(),hashlib.sha256(h+p).hexdigest()
def split(a,field):
    s=a[field['offset']:field['offset']+field['count']]
    return s.reshape(field['shape']) if field['shape'] else s.copy()
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--run',type=Path,default=DEFAULT_RUN); ap.add_argument('--output',type=Path,default=HERE/'real_lio_rows_v12_format16');args=ap.parse_args()
    run=args.run.resolve(); out=args.output.resolve()
    if not out.is_relative_to(HERE): raise ValueError('Only owned evaluation output is allowed')
    stats=json.loads((run/'fastlivo_diagnostics/writer_stats.json').read_text())
    if not stats.get('final') or stats.get('dropped') or stats.get('writer_io_failed'): raise ValueError('Source writer is not complete and lossless')
    snapshots=json.loads((run/'navigation_source_snapshots.json').read_text())
    schema_ref=next(v for k,v in snapshots.items() if k.endswith('/lio_diagnostic_schema.json'))
    schema_path=Path(schema_ref['snapshot']).resolve()
    if not schema_path.is_relative_to(run/'sources') or sha(schema_path)!=schema_ref['sha256']: raise ValueError('Schema source binding failed')
    schema=json.loads(schema_path.read_text()); fields={x['name']:x for x in schema['kinds']['100']['fields']}
    config_path=run/'navigation_fastlivo.yaml';config=json.loads(config_path.read_text());extr=config['/**']['ros__parameters']['extrin_calib']
    extR=np.asarray(extr['extrinsic_R'],dtype='<f8').reshape(3,3);extT=np.asarray(extr['extrinsic_T'],dtype='<f8')
    index=run/'fastlivo_diagnostics/index.csv';binary=run/'fastlivo_diagnostics/records.bin'
    aggregates={}; candidates=[]
    with index.open(newline='')as f:
        for row in csv.DictReader(f):
            kind=int(row['kind'])
            if kind==100:
                if key(row) in aggregates: raise ValueError('Duplicate aggregate identity')
                aggregates[key(row)]=row
            elif kind==101:
                n=(int(row['n_values'])-3)//107
                if n<=0 or 3+n*107!=int(row['n_values']): raise ValueError('Malformed constraint length')
                candidates.append((n,row))
    if not candidates: raise ValueError('No real constraints')
    # Predeclared deterministic first/min/max, ties retain source order.
    selections=[('first',candidates[0]),('minimum',min(candidates,key=lambda x:x[0])),('maximum',max(candidates,key=lambda x:x[0]))]
    out.mkdir(parents=True,exist_ok=False)
    exported=[]
    with binary.open('rb')as f:
        if f.read(16)!=MAGIC: raise ValueError('Unsupported binary magic')
        for label,(n,row) in selections:
            aggregate=aggregates.get(key(row))
            if aggregate is None: raise ValueError('Missing exact same-iteration aggregate')
            data,record_sha=read_record(f,row);a,aggregate_sha=read_record(f,aggregate)
            if data[0]!=1 or data[1]!=n or data[2]!=107 or len(a)!=1393 or a[0]!=1 or a[2]!=n: raise ValueError('Schema/count mismatch')
            rows=data[3:].reshape(n,107)
            inputs={'rows':rows,'prop_state':split(a,fields['state_propagat']),'before_state':split(a,fields['state_iter_before']),'before_cov':split(a,fields['P_before']),'extR':extR,'extT':extT}
            expected={'expected_H':rows[:,35:41],'expected_R_inv':rows[:,41],'expected_meas':rows[:,42],'expected_sigma':rows[:,43],'expected_body_world':rows[:,89:98].reshape(n,3,3),'expected_HTH':split(a,fields['M6']),'expected_HTz':split(a,fields['HTz6']),'aggregate100':a,'constraint101':data}
            path=out/f'{label}_{n}_rows.npz'
            with path.open('xb')as dest: np.savez(dest,**inputs,**expected)
            # Native C++ readers may consume this exact uncompressed fixed shape format.
            native=out/f'{label}_{n}_rows.bin'
            with native.open('xb')as dest:
                dest.write(b'FLIVOJACROW0001\0');dest.write(struct.pack('<Q',n))
                for name in ('extR','extT','prop_state','before_state','before_cov','rows'): dest.write(inputs[name].astype('<f8',copy=False).tobytes(order='C'))
            exported.append(dict(selection=label,rows=n,npz=str(path),npz_sha256=sha(path),native_binary=str(native),native_binary_sha256=sha(native),record_index=dict(row),record_sha256=record_sha,aggregate_index=dict(aggregate),aggregate_record_sha256=aggregate_sha,array_shapes={k:list(v.shape)for k,v in inputs.items()},all_input_values_finite=all(np.isfinite(v).all()for v in inputs.values())))
    manifest=dict(schema='real_logged_lio_jacobian_component_fixture/v1',created_UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_run=str(run),source_index_sha256=sha(index),source_binary_stat={'size':binary.stat().st_size,'mtime_ns':binary.stat().st_mtime_ns},source_binary_entire_hash_recomputed=False,selected_record_bytes_hashed=True,source_schema_sha256=sha(schema_path),source_schema=str(schema_path),source_navigation_configuration_sha256=sha(config_path),source_navigation_configuration=str(config_path),source_writer_stats_sha256=sha(run/'fastlivo_diagnostics/writer_stats.json'),authoritative_detail_window_s=[stats['detail_begin_s'],stats['detail_end_s']],row_order='Original kind101 order; no sorting, no numerical reconstruction',state_order=schema['state25_order'],native_binary_format='16-byte FLIVOJACROW0001\\0 magic; uint64 N; float64 row-major extR9,extT3,prop_state25,before_state25,before_cov361,rowsNx107',scope='Exact selected production-row component inputs and expected logged outputs. Neither full residual matching, complete voxel map, nor estimator trajectory replay.',selections=exported,exporter_sha256=sha(Path(__file__)))
    with(out/'manifest.json').open('x')as f: f.write(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'manifest':str(out/'manifest.json'),'manifest_sha256':sha(out/'manifest.json'),'fixtures':[{'selection':v['selection'],'rows':v['rows'],'npz':v['npz'],'native_binary':v['native_binary']}for v in exported]}))
if __name__=='__main__':main()
