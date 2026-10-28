"""Read-only comparison material for independent builds; never a startup gate."""
from pathlib import Path
import argparse,hashlib,json,struct
FILES=['install/dds/lib/libfastrtps.so.2.14.6','install/ros/lib/libfast_livo2_core.so',
 'install/ros/lib/fast_livo2_ros/fastlivo_mapping','install/ros/lib/scan_planner/scan_planner_node',
 'install/actuators/lib/libteacher_actuator.so','install/communication/lib/writer_qos.so']
def sections(p):
    data=p.read_bytes()
    if data[:6]!=b'\x7fELF\x02\x01':raise ValueError('Expected little-endian ELF64')
    offset=struct.unpack_from('<Q',data,40)[0];size,count,names=struct.unpack_from('<HHH',data,58)
    entries=[struct.unpack_from('<IIQQQQIIQQ',data,offset+i*size)for i in range(count)]
    name_table=entries[names];strings=data[name_table[4]:name_table[4]+name_table[5]];result={}
    for e in entries:
        name=strings[e[0]:].split(b'\0',1)[0].decode()
        if name in ('.text','.rodata','.data','.eh_frame'):
            result[name]=dict(bytes=e[5],sha256=hashlib.sha256(data[e[4]:e[4]+e[5]]).hexdigest())
    return result
def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path.cwd());a=p.parse_args()
    files=sorted(set(FILES)|{str(p.relative_to(a.root)) for p in (a.root/'install/ros/lib').glob('*.so')})
    print(json.dumps({f:sections(a.root/f) for f in files},indent=2))
if __name__=='__main__':main()
