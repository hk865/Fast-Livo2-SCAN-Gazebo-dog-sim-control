#!/usr/bin/env python3
from pathlib import Path
import json
from fixture_compile_helper import compile_fixture,sha
OUT=Path(__file__).resolve().parent;base=OUT.parents[2]
result={'schema':'V18_fresh_VIO_fixture_build/v1','variants':{},'source_sha256':sha(OUT/'vio_patch_fixture.cpp')}
for name,ws in [('baseline_V12',base/'navigation/lidar_sampling_v12/slam_ws'),('candidate_V18',base/'navigation/combined_compute_v18/slam_ws')]:
 d=OUT/name;d.mkdir(exist_ok=False)
 env,m=compile_fixture(ws,d,OUT/'vio_patch_fixture.cpp');env['FASTLIVO_LIO_JACOBIAN_THREADS']='4';m['env']=env;result['variants'][name]=m
(OUT/'vio_fixture_build_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
