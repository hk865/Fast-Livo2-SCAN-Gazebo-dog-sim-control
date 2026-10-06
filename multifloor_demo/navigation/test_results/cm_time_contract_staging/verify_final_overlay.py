#!/usr/bin/env python3
"""Authorized final isolated CM verification; no Gazebo or motion outputs."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

HERE=Path(__file__).resolve().parent
DEMO=HERE.parents[2]
WORKSPACE=DEMO/'simulation/ros2_control_ws'
PACKAGE=WORKSPACE/'src/controller_manager'
LIB=WORKSPACE/'install/controller_manager/lib/libcontroller_manager.so'
BASE=Path('/opt/ros/jazzy/lib/libcontroller_manager.so')
FIXTURE=HERE/'build_fixture/cm_time_contract_fixture'
OUT=HERE/'final_adoption_v2'
OUT.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def command(args):
    result=subprocess.run(args,capture_output=True,text=True,check=False)
    return dict(argv=args,return_code=result.returncode,stdout=result.stdout,stderr=result.stderr)


def exported_cm_functions(library):
    data=command(['nm','-D','--defined-only','--demangle',str(library)])
    (OUT/(library.parent.parent.name+'_nm.txt')).write_text(data['stdout']+data['stderr'])
    functions=set()
    for line in data['stdout'].splitlines():
        parts=line.split(None,2)
        if len(parts)==3 and parts[1]=='T' and parts[2].startswith('controller_manager::'):
            functions.add(parts[2])
    return functions


baseline_functions=exported_cm_functions(BASE)
final_functions=exported_cm_functions(LIB)
baseline_pkg=HERE/'baseline/controller_manager'
source_differences={}
for path in baseline_pkg.rglob('*'):
    if not path.is_file():continue
    target=PACKAGE/path.relative_to(baseline_pkg)
    if not target.exists() or sha(target)!=sha(path):
        source_differences[str(path.relative_to(baseline_pkg))]=dict(baseline_sha256=sha(path),final_sha256=sha(target) if target.exists() else None)
baseline_cpp=baseline_pkg/'src/controller_manager.cpp'
final_cpp=PACKAGE/'src/controller_manager.cpp'
patch=''.join(difflib.unified_diff(baseline_cpp.read_text().splitlines(True),final_cpp.read_text().splitlines(True),fromfile='official_4.45.2/controller_manager.cpp',tofile='Demo/controller_manager.cpp'))
(OUT/'final_minimal.patch').write_text(patch)
headers={str(p.relative_to(PACKAGE/'include')):dict(final_sha256=sha(p),installed_sha256=sha(Path('/opt/ros/jazzy/include/controller_manager')/p.relative_to(PACKAGE/'include')),equal=sha(p)==sha(Path('/opt/ros/jazzy/include/controller_manager')/p.relative_to(PACKAGE/'include'))) for p in (PACKAGE/'include').rglob('*.hpp')}
reloc=command(['ldd','-r',str(LIB)])
(OUT/'final_ldd_relocations.txt').write_text(reloc['stdout']+reloc['stderr'])
dependencies=[]
for line in reloc['stdout'].splitlines():
    match=re.search(r'(?:=>\s+)?(/\S+)\s+\(',line)
    if match:
        path=Path(match.group(1))
        dependencies.append(dict(path=str(path),resolved_path=str(path.resolve()),sha256=sha(path)))
compiler=command(['/usr/bin/c++','--version'])
cache=WORKSPACE/'build/controller_manager/CMakeCache.txt'
commands=WORKSPACE/'build/controller_manager/compile_commands.json'
(OUT/'compile_commands.json').write_bytes(commands.read_bytes())
cache_fields={line.split('=',1)[0]:line.split('=',1)[1] for line in cache.read_text().splitlines() if '=' in line and not line.startswith('//') and re.match(r'(CMAKE_(CXX_COMPILER|C_COMPILER|BUILD_TYPE|CXX_FLAGS)|BUILD_TESTING|.*_DIR):',line)}
build_log=WORKSPACE/'log/latest_build/controller_manager/stdout_stderr.log'
(OUT/'build_stdout_stderr.log').write_bytes(build_log.read_bytes())

# Only this fixture's local /clock override; no publisher or motion interface.
env=os.environ.copy();env['ROS_DOMAIN_ID']='77';env['ROS_LOG_DIR']=str(OUT/'ros_logs')
env['LD_LIBRARY_PATH']=str(LIB.parent)+':'+env.get('LD_LIBRARY_PATH','')
start=time.monotonic_ns()
with (OUT/'controlled_final.log').open('w') as log:
    process=subprocess.Popen([str(FIXTURE),'candidate'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    pgid=os.getpgid(process.pid)
    try:
        exit_code=process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(pgid,signal.SIGINT)
        try:exit_code=process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(pgid,signal.SIGKILL);exit_code=process.wait(timeout=5)
end=time.monotonic_ns()
text=(OUT/'controlled_final.log').read_text()
checks=[line for line in text.splitlines() if line.startswith('CHECK ')]
loaded=[line for line in text.splitlines() if line.startswith('LOADED_CM ')]
ps=command(['ps','-o','pid=,args=','-g',str(pgid)])
ready=(exit_code==0 and len(checks)==12 and all(c.endswith(' PASS') for c in checks)
       and loaded==['LOADED_CM '+str(LIB)] and not ps['stdout'].strip()
       and not (baseline_functions-final_functions) and not (final_functions-baseline_functions)
       and all(p['equal'] for p in headers.values())
       and list(source_differences)==['src/controller_manager.cpp']
       and reloc['return_code']==0 and 'undefined symbol' not in reloc['stdout']+reloc['stderr'])
receipt=dict(scope=__doc__,ready=ready,workspace=str(WORKSPACE),library=dict(path=str(LIB),sha256=sha(LIB),baseline_path=str(BASE),baseline_sha256=sha(BASE)),
    build=dict(command='colcon --log-base log build --base-paths src/controller_manager --build-base build --install-base install --packages-select controller_manager --executor sequential --allow-overriding controller_manager --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON',exit_code=0,
        initial_environment='Unset old AMENT/CMAKE/COLCON/LD prefixes, source /opt/ros/jazzy only; final build does not need excluded preparer',compiler=compiler,cache_fields=cache_fields,cache_sha256=sha(cache),compile_commands_sha256=sha(commands),build_log_sha256=sha(build_log)),
    source=dict(upstream_version='4.45.2',official_archive_sha256=sha(HERE/'ros2_control-4.45.2.tar.gz'),preparer_sha256=sha(HERE/'prepare_overlay.py'),
        differences=source_differences,package_files_sha256={str(p.relative_to(PACKAGE)):sha(p) for p in PACKAGE.rglob('*') if p.is_file()},
        license_sha256=sha(PACKAGE/'LICENSE'),standard_license_collected=True,patch_sha256=sha(OUT/'final_minimal.patch')),
    ABI=dict(public_headers=headers,baseline_CM_function_count=len(baseline_functions),final_CM_function_count=len(final_functions),removed=sorted(baseline_functions-final_functions),added=sorted(final_functions-baseline_functions),undefined_relocations=False,
        certificate_scope='Public headers and strong exported CM functions preserved; actual installed-dependency controlled synchronous fixture, not a universal ABI proof'),
    actual_dependencies=dependencies,
    fixture=dict(domain=77,binary=str(FIXTURE),binary_sha256=sha(FIXTURE),source_sha256=sha(HERE/'fixture/cm_time_contract_fixture.cpp'),pid=process.pid,pgid=pgid,start_wall_monotonic_ns=start,end_wall_monotonic_ns=end,exit_code=exit_code,checks=checks,loaded_CM=loaded,
        requested_lower_rate_hz=100,actual_configured_lower_rate_hz=83,control_initialization='Explicit ControllerSpec matches load_controller clock initialization; actual sync ControllerInterface callback, no load_controller plugin call',owned_cleanup_ps=ps,owned_group_clean=not ps['stdout'].strip(),log_sha256=sha(OUT/'controlled_final.log')),
    limitations=['Existing negative/backwards physical time not hidden; no mid-run world reset support.', 'Sim time input monotonic and synchronous JTC/JSB scope; non-sim branch kept unchanged.', 'CM diagnostics remain on node ROSclock. No claim to repair /clock transport itself.', 'No body command, gait, PID, region, arrival or protection changed by this overlay.', 'Final DSO rebuilt at final path; SHA may differ from excluded candidate due build paths.'])
(OUT/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(dict(ready=ready,library=receipt['library'],CM_functions=(len(baseline_functions),len(final_functions)),checks=checks,owned_clean=receipt['fixture']['owned_group_clean']),indent=2))
raise SystemExit(not ready)
