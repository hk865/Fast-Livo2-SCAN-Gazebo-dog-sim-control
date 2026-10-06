from pathlib import Path
import hashlib,json
art=Path(__file__).resolve().parent;base=art.parents[2]
old=base/'navigation/lidar_sampling_v12/slam_ws/src/fast_livo2_core';new=base/'navigation/parallel_lio_v15/slam_ws/src/fast_livo2_core'
def remove_tag(s,tag,indent='',extra_newline=False):
 begin=indent+'// LIO_JACOBIAN_PARALLEL_BEGIN '+tag+'\n';end=indent+'// LIO_JACOBIAN_PARALLEL_END '+tag+'\n'
 a=s.index(begin);b=s.index(end,a)+len(end)+(1 if extra_newline else 0);return s[:a]+s[b:]
s=(new/'src/voxel_map.cpp').read_text();tag='production_rows_call';a=s.index('// LIO_JACOBIAN_PARALLEL_BEGIN '+tag+'\n');end='// LIO_JACOBIAN_PARALLEL_END '+tag+'\n';b=s.index(end,a)+len(end);s=s[:a]+(art/'original_v12_jacobian_loop.txt').read_text()+s[b:]
s=remove_tag(s,'production_rows_method',extra_newline=True);s=remove_tag(s,'configuration')
h=remove_tag((new/'include/fast_livo2_core/core/voxel_map.h').read_text(),'production_rows_interface','  ')
assert s.encode()==(old/'src/voxel_map.cpp').read_bytes();assert h.encode()==(old/'include/fast_livo2_core/core/voxel_map.h').read_bytes()
print(json.dumps({'cpp_reconstructed_exact':True,'header_reconstructed_exact':True,'original_loop_sha256':hashlib.sha256((art/'original_v12_jacobian_loop.txt').read_bytes()).hexdigest()}))
