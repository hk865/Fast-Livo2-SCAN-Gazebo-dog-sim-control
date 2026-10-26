#!/usr/bin/env python3
"""Build the pinned, privately installed Fast DDS 2.14.6 backport. No downloads.

Without --execute, validate archive pins and print the complete build plan.
Every build uses fresh source/build/prefix directories, never a reused CMake cache.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tarfile
sys.dont_write_bytecode = True
from common import (ROOT, MATERIALS, allocated_sections, elf_identity, private_path,
                    read, sha, write_new)


def extract(archive, destination, expected_root):
    with tarfile.open(archive, 'r:gz') as source:
        for member in source.getmembers():
            target = destination / member.name
            if (not target.resolve().is_relative_to(destination) or
                    member.name.split('/')[0] != expected_root or member.isdev()):
                raise ValueError('Unsafe/unexpected source archive member')
        source.extractall(destination, filter='data')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fastdds-archive', type=Path, required=True)
    parser.add_argument('--asio-archive', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, default=ROOT/'.local_build/private_dds')
    parser.add_argument('--prefix', type=Path, default=ROOT/'.local/private_dds/install')
    parser.add_argument('--ros-prefix', type=Path, default=Path('/opt/ros/jazzy'))
    parser.add_argument('--gtest-source', type=Path, default=Path('/usr/src/googletest'))
    parser.add_argument('--gtest-dir', type=Path, help='Existing private/system GTest CMake directory instead of building the local source')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--baseline', action='store_true', help='Build unpatched library with the same regression fixtures')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    pins = read(MATERIALS/'SOURCES.json')
    for role, archive in [('fastdds', args.fastdds_archive), ('asio', args.asio_archive)]:
        if sha(archive) != pins[role]['archive_sha256']:
            raise ValueError('Pinned source archive SHA mismatch: ' + role)
    for name, expected in pins['backport']['patch_sha256'].items():
        if sha(MATERIALS/name) != expected:
            raise ValueError('Backport/test patch changed: ' + name)
    work, prefix = private_path(args.work_dir), private_path(args.prefix)
    if work.exists() or prefix.exists() or work.is_relative_to(prefix) or prefix.is_relative_to(work):
        raise ValueError('Separate, fresh private work and install directories required')
    ros = args.ros_prefix.resolve(strict=True)
    source = work/pins['fastdds']['archive_root']
    asio = work/pins['asio']['archive_root']/'asio/include'
    build = work/'build'
    gtest = args.gtest_dir.resolve(strict=True) if args.gtest_dir else work/'gtest-install/lib/cmake/GTest'
    steps = []
    if not args.gtest_dir:
        local_gtest = args.gtest_source.resolve(strict=True)
        steps += [['cmake', '-S', str(local_gtest), '-B', str(work/'gtest-build'),
                   '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX='+str(work/'gtest-install'),
                   '-DBUILD_GMOCK=ON', '-DINSTALL_GTEST=ON'],
                  ['cmake', '--build', str(work/'gtest-build'), '--parallel', str(args.jobs)],
                  ['cmake', '--install', str(work/'gtest-build')]]
    patches = ([] if args.baseline else [pins['backport']['production_patch']]) + pins['backport']['test_patches']
    steps += [['patch', '--batch', '--forward', '-p1', '-d', str(source), '-i', str(MATERIALS/name)] for name in patches]
    configure = ['cmake', '-S', str(source), '-B', str(build),
                 '-DCMAKE_BUILD_TYPE=RelWithDebInfo', '-DCMAKE_INSTALL_PREFIX='+str(prefix),
                 '-DCMAKE_PREFIX_PATH='+str(ros), '-DAsio_INCLUDE_DIR='+str(asio),
                 '-DTHIRDPARTY=OFF', '-DTHIRDPARTY_UPDATE=OFF', '-DSECURITY=ON',
                 '-DFASTDDS_STATISTICS=ON', '-DSQLITE3_SUPPORT=ON', '-DLOG_NO_INFO=ON',
                 '-DSHM_TRANSPORT_DEFAULT=ON', '-DCOMPILE_EXAMPLES=OFF', '-DCOMPILE_TOOLS=ON',
                 '-DEPROSIMA_BUILD_TESTS=ON', '-DSYSTEM_TESTS=OFF', '-DPERFORMANCE_TESTS=OFF',
                 '-DGTest_DIR='+str(gtest)]
    steps += [configure, ['cmake', '--build', str(build), '--target', 'fastrtps',
                         'SharedMemTests', 'fast-discovery-server', '--parallel', str(args.jobs)],
              ['cmake', '--install', str(build)]]
    plan = dict(schema='private_fastdds_build/v1', execute=args.execute,
                variant='unpatched' if args.baseline else 'patched',
                source_pins=pins, commands=steps, prefix=str(prefix), work_dir=str(work),
                no_download=True, no_system_install=True,
                test_command=[str(build/'test/unittest/transport/SharedMemTests'),
                              '--gtest_filter=SHMTransportTests.reviewed_heartbeat_*'])
    if not args.execute:
        import json
        print(json.dumps(plan, indent=2))
        return
    work.mkdir(parents=True)
    write_new(work/'BUILD_PLAN.json', plan)
    extract(args.fastdds_archive, work, pins['fastdds']['archive_root'])
    extract(args.asio_archive, work, pins['asio']['archive_root'])
    for index, command in enumerate(steps):
        with (work/('step_%02d.log' % index)).open('x') as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    library = (prefix/'lib/libfastrtps.so.2.14.6').resolve(strict=True)
    before = allocated_sections(library)
    full_sha = sha(library)
    debug = library.parent/'.debug'/('libfastrtps.so.2.14.6.debug')
    debug.parent.mkdir()
    subprocess.run(['objcopy', '--only-keep-debug', str(library), str(debug)], check=True)
    subprocess.run(['objcopy', '--strip-debug', str(library)], check=True)
    subprocess.run(['objcopy', '--add-gnu-debuglink='+str(debug), str(library)], check=True)
    if allocated_sections(library) != before:
        raise ValueError('Split-debug changed allocated runtime sections')
    cache = (build/'CMakeCache.txt').read_text()
    compiler = next(line.split('=',1)[1] for line in cache.splitlines()
                    if line.startswith('CMAKE_CXX_COMPILER:FILEPATH='))
    write_new(work/'BUILD_IDENTITY.json', dict(schema='private_fastdds_identity/v1',
        library=elf_identity(library), full_debug_sha256=full_sha, debug_file=str(debug),
        debug_sha256=sha(debug), allocated_sections_identical=True,
        compiler_path=compiler,
        compiler=subprocess.run([compiler, '--version'], check=True, capture_output=True, text=True).stdout,
        source_pins=pins, patches_applied=patches, cmake_cache_sha256=sha(build/'CMakeCache.txt')))
    print('Private DDS built; tests are built, not executed: ' + str(work/'BUILD_IDENTITY.json'))


if __name__ == '__main__':
    main()
