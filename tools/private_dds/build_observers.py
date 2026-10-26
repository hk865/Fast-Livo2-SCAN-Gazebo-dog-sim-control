#!/usr/bin/env python3
"""Build creation witness and bounded native owner probe against one private ELF.

No DDS participant/process is started. GDB is file-only, without an inferior.
Unknown layouts, missing symbols or ambiguous health-call sites fail closed.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode = True
from common import ROOT, artifact, elf_identity, private_path, read, run, sha, write_new

HERE = Path(__file__).resolve().parent
EXPECTED_LAYOUT = dict(reader_participant_pointer_offset=8, reader_mutex_A_offset=0x188,
    participant_manager_pointer_offset=0x7d8, manager_mutex_M_offset=0,
    manager_I_pointer_offset=0x98, participant_mutex_S_offset=0x53c8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--fastdds-source', type=Path, required=True,
                        help='Exact patched source tree used to build this library (private headers)')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'.local_build/private_dds-observers')
    parser.add_argument('--ros-prefix', type=Path, default=Path('/opt/ros/jazzy'))
    parser.add_argument('--cxx', default='c++')
    parser.add_argument('--execute', action='store_true', help='Compile; otherwise print the plan only')
    args = parser.parse_args()
    identity = elf_identity(args.library)
    library = Path(identity['path'])
    source = args.fastdds_source.resolve(strict=True)
    public = library.parent.parent/'include'
    headers = [public, source/'src/cpp', source/'include']
    if not all(path.is_dir() for path in headers):
        raise ValueError('Exact installed/private DDS header roots required')
    ros = args.ros_prefix.resolve(strict=True)
    if not (ros/'include/rcl/rcl/publisher.h').is_file():
        raise ValueError('ROS rcl development headers unavailable')
    output = private_path(args.output_dir)
    if output.exists():
        raise ValueError('Fresh observer output directory required')
    # Same ROS include discovery as the original observer build; private DDS
    # public/private headers always precede every ROS underlay include.
    ros_headers = sorted(p for p in (ros/'include').iterdir() if p.is_dir())
    witness = output/'participant_witness.so'
    probe = output/'owner_wait_probe.so'
    common = ['-std=c++17', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra']
    witness_cmd = [args.cxx, *common, '-pthread', '-I'+str(public),
        '-I'+str(ros/'include/fastcdr'), str(HERE/'participant_witness.cpp'),
        '-Wl,--no-undefined', '-Wl,-z,relro', '-ldl', '-lcrypto', '-o', str(witness)]
    probe_cmd = [args.cxx, *common, '-Wno-unused-parameter', '-I'+str(output),
        *['-I'+str(p) for p in [*headers, *ros_headers]], str(HERE/'owner_wait_probe.cpp'),
        '-o', str(probe), '-L'+str(library.parent), '-Wl,-rpath,'+str(library.parent),
        '-L'+str(ros/'lib'), '-Wl,-rpath,'+str(ros/'lib'),
        '-Wl,--no-as-needed', '-lfastrtps', '-lrcl', '-Wl,--as-needed', '-ldl', '-pthread']
    plan = dict(schema='private_dds_observer_build/v1', library=identity,
        witness_command=witness_cmd, probe_command=probe_cmd, source_root=str(source),
        private_headers=[str(p) for p in headers], no_DDS_or_ROS_process_started=True,
        sources={str(HERE/n):sha(HERE/n) for n in ['participant_witness.cpp',
                  'owner_wait_probe.cpp', 'inspect_layout.py', 'dwarf_layout.gdb.py']})
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return
    output.mkdir(parents=True)
    write_new(output/'BUILD_PLAN.json', plan)
    layout_path = output/'probe_layout.json'
    subprocess.run([sys.executable, '-B', str(HERE/'inspect_layout.py'), str(library),
        '--out', str(layout_path), '--expected-sha256', identity['sha256'],
        '--expected-build-id', identity['build_id'], '--probe-source', str(HERE/'owner_wait_probe.cpp')], check=True)
    layout = read(layout_path)
    if layout['layout'] != EXPECTED_LAYOUT:
        raise ValueError('Unsupported private-member/native-lock ABI; no old offsets may be reused')
    calls = layout['health']['nanosleep_calls']
    if len(calls) != 1 or not any('SharedMemGlobal.hpp' in text for text in calls[0]['source_inlining']):
        raise ValueError('One exact source-proven healthy_check nanosleep call is required')
    symbols = layout['symbols']
    header = ['// Generated from this exact ELF; do not copy between builds.',
        '#define PRIVATE_DDS_LIBRARY_PATH '+json.dumps(str(library)),
        '#define PRIVATE_DDS_BUILD_ID_BYTES '+','.join('0x'+identity['build_id'][i:i+2] for i in range(0,40,2)),
        '#define PRIVATE_DDS_HEALTH_RETURN_PC '+hex(calls[0]['return_PC'])]
    for macro, role in [('SHM','SHMSEND'), ('ACK','ACK2'), ('SYNC','SYNC'), ('GETBUF','GETBUF')]:
        header.append('#define PRIVATE_DDS_'+macro+'_OFFSET '+hex(symbols[role]['ELF_offset']))
    (output/'probe_identity.hpp').write_text('\n'.join(header)+'\n')
    for name, command in [('witness', witness_cmd), ('probe', probe_cmd)]:
        with (output/(name+'.log')).open('x') as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    dynamic = run(['readelf', '-d', witness])
    if any(x in dynamic for x in ['[libfastrtps', '[libfastdds']):
        raise ValueError('Witness unexpectedly links DDS; output retained for inspection')
    if str(library).encode() not in probe.read_bytes() or bytes.fromhex(identity['build_id']) not in probe.read_bytes():
        raise ValueError('Compiled observer is missing exact path/BuildID guard')
    plan.update(participant_witness=artifact(witness), owner_probe=artifact(probe),
                layout_sha256=sha(layout_path), generated_header_sha256=sha(output/'probe_identity.hpp'),
                witness_dynamic=dynamic)
    write_new(output/'OBSERVER_BUILD.json', plan)
    print('Observers built, not preloaded or executed: '+str(output/'OBSERVER_BUILD.json'))


if __name__ == '__main__':
    main()
