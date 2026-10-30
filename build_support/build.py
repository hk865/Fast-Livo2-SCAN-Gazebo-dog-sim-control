#!/usr/bin/env python3
"""Build all project-native dependencies in fresh project-local directories.

Downloads use official upstream source URLs. Content hashes and runtime
source/model/binary identity contracts are not used.
"""
from pathlib import Path
import argparse,json,os,subprocess,sys,tarfile,urllib.request
ROOT=Path(__file__).resolve().parents[1]
def run(command,**kw):
    print('+',' '.join(map(str,command)),flush=True)
    subprocess.run(list(map(str,command)),check=True,**kw)
def download(url,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    if not target.exists():
        with urllib.request.urlopen(url,timeout=90) as src,target.with_suffix('.part').open('wb') as out:
            while data:=src.read(1024*1024):out.write(data)
        target.with_suffix('.part').replace(target)
    return dict(url=url,bytes=target.stat().st_size)

def unpack(archive,directory):
    directory.mkdir(parents=True,exist_ok=True)
    with tarfile.open(archive) as t:t.extractall(directory,filter='data')
def cmake(source,build,prefix,jobs,extra=()):
    run(['cmake','-S',source,'-B',build,'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(prefix),*extra])
    run(['cmake','--build',build,'--parallel',jobs]);run(['cmake','--install',build])
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--jobs',type=int,default=2)
    ap.add_argument('--skip-dds',action='store_true',help='Reuse this copy’s already built patched DDS, never the host library')
    a=ap.parse_args()
    if not 1<=a.jobs<=4:ap.error('Use 1..4 build jobs to limit load')
    work=ROOT/'build';install=ROOT/'install';sources=work/'sources';sources.mkdir(parents=True,exist_ok=True)
    receipts={};pins=json.loads((ROOT/'third_party/fastdds/SOURCES.json').read_text())
    for name in ('fastdds','asio'):
        pin=pins[name];archive=sources/(name+'.tar.gz')
        receipts[name]=download(pin['archive_url'],archive)
    dds=install/'dds'
    if not a.skip_dds:
        run([sys.executable,ROOT/'build_support/private_dds/build_private_dds.py',
            '--fastdds-archive',sources/'fastdds.tar.gz','--asio-archive',sources/'asio.tar.gz',
            '--work-dir',work/'dds','--prefix',dds,'--jobs',a.jobs,'--execute'])
        run([work/'dds/build/test/unittest/transport/SharedMemTests',
             '--gtest_filter=SHMTransportTests.reviewed_heartbeat_*'])
    if not(dds/'lib/libfastrtps.so.2.14.6').exists():raise FileNotFoundError('Patched DDS build is required')
    # Official v1.3.1 supplies Mid360s, referenced by this bundled ROS driver.
    commit='f5d9375f84efe2b15bc0a052d3e18482ed13adf4'
    url='https://codeload.github.com/Livox-SDK/Livox-SDK2/tar.gz/'+commit
    archive=sources/('livox-sdk2-'+commit+'.tar.gz')
    receipts['livox_sdk2']=download(url,archive)
    sdk=sources/('Livox-SDK2-'+commit)
    if not sdk.exists():unpack(archive,sources)
    # v1.2.5 uses std::uint*_t without including its declaring standard header.
    # GCC13 no longer supplies that transitive include. No SDK logic changes.
    header=sdk/'sdk_core/comm/define.h';text=header.read_text()
    if '#include <cstdint>' not in text:
        header.write_text(text.replace('#define LIVOX_DEFINE_H_','#define LIVOX_DEFINE_H_\n#include <cstdint>',1))
    receipts['livox_sdk2']['local_build_fix']='explicit cstdint declarations for GCC13 (comm header and forced standard include for remaining upstream headers)'
    cmake(sdk,work/('livox-'+commit[:8]),install/'livox',a.jobs,['-DBUILD_SHARED_LIBS=ON','-DCMAKE_CXX_FLAGS=-include cstdint'])
    env=dict(os.environ)
    env['CMAKE_PREFIX_PATH']=':'.join([str(install/'livox'),str(dds),env.get('CMAKE_PREFIX_PATH','')])
    env['LD_LIBRARY_PATH']=':'.join([str(dds/'lib'),str(install/'livox/lib'),env.get('LD_LIBRARY_PATH','')])
    env['MAKEFLAGS']='-j'+str(a.jobs)
    run(['colcon','--log-base',work/'logs','build','--merge-install','--executor','sequential',
        '--base-paths',ROOT/'src/sdk',ROOT/'src/localization/frontend',ROOT/'src/planning/scan',
        '--build-base',work/'ros','--install-base',install/'ros',
        '--cmake-args','-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF',
        '-DROS_EDITION=ROS2','-DDISTRO_ROS=jazzy','-DV16_VIO_PATCH_THREADS=4'],env=env,cwd=ROOT)
    cmake(ROOT/'src/simulation/actuators',work/'actuators',install/'actuators',a.jobs)
    cmake(ROOT/'src/common/communication',work/'communication',install/'communication',a.jobs,
          ['-DCMAKE_PREFIX_PATH='+str(dds)+';'+os.environ.get('CMAKE_PREFIX_PATH','').replace(':',';')])
    cmake(ROOT/'src/localization/backend/native',work/'backend_graph',install/'backend',a.jobs)
    (install/'pct').mkdir(parents=True,exist_ok=True)
    run(['g++','-std=c++17','-O2','-I/usr/include/eigen3','-I'+str(ROOT/'src/planning/pct/native'),
        ROOT/'src/planning/pct/pct_cli.cpp',ROOT/'src/planning/pct/native/a_star/a_star_search.cc',
        '-o',install/'pct/pct_astar'])
    packages=subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\n'],capture_output=True,text=True,check=True).stdout
    (install/'system_packages.tsv').write_text(packages)
    (install/'build_sources.json').write_text(json.dumps(receipts,indent=2)+'\n')
    print('All native components built; navigation has not been validated by this command.')
if __name__=='__main__':main()
