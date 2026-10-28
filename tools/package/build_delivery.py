#!/usr/bin/env python3
"""Create local delivery archives; never publish or import the application."""
from pathlib import Path
import hashlib, json, os, re, tarfile, zipfile

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output'
OMIT = {'.git', '.local', 'build', 'install', 'log', 'runs', 'output', 'implementation', '__pycache__', 'historical_profiles', 'legacy_notes'}
REQUIRED = [
    'start.py', 'Dockerfile', 'build_support/build.py',
    'models/student_model_4999_deterministic_jit.pt',
    'assets/meshes/L1_4D_Lidar.dae', 'assets/meshes/trunk.dae',
    'assets/world/go2_measured.urdf', 'assets/world/three_floors.sdf',
    'assets/geometry/route.json',
    'src/bringup/launch/demo.launch.py',
    'src/localization/frontend/fast_livo2_core/src/LIVMapper.cpp',
    'src/planning/scan/plan_manage/src/planner_manager.cpp',
    'src/control/policy/student.py', 'src/simulation/actuators/teacher_actuator.cpp',
    'third_party/fastdds/heartbeat_2_14_6.patch',
]


def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''): h.update(chunk)
    return h.hexdigest()


def source_files():
    result = []
    for base, dirs, files in os.walk(ROOT):
        dirs[:] = sorted(x for x in dirs if x not in OMIT)
        for name in sorted(files):
            p = Path(base) / name
            if p.suffix in {'.pyc', '.so'}: continue
            if p.is_symlink(): raise RuntimeError('Source symlink requires explicit review: ' + str(p.relative_to(ROOT)))
            if any(x in name.lower() for x in ('.pem', '.key', 'credentials', 'id_rsa')):
                raise RuntimeError('Credential-like filename: ' + str(p.relative_to(ROOT)))
            if p.suffix in {'.py', '.md', '.json', '.xml', '.txt', '.sh', '.cmake', '.cpp', '.h', '.hpp', '.yaml', '.yml'}:
                s = p.read_text(errors='replace')
                if re.search(r'/home/[a-zA-Z0-9_-]+/(?:projects|workspace)/|navigation_runtime_20\d{6}|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|AKIA[A-Z0-9]{16}', s):
                    raise RuntimeError('Private host reference or secret marker: ' + str(p.relative_to(ROOT)))
            result.append(p)
    names = {p.relative_to(ROOT).as_posix() for p in result}
    if set(REQUIRED) - names: raise RuntimeError('Missing required source/assets: ' + repr(sorted(set(REQUIRED) - names)))
    return result


def main():
    OUT.mkdir(exist_ok=True)
    files = source_files()
    manifest = {'purpose': 'delivery integrity only; never a runtime gate',
                'files': {p.relative_to(ROOT).as_posix(): {'bytes': p.stat().st_size, 'sha256': digest(p)} for p in files}}
    source = OUT / 'go2-navigation-source-assets.zip'
    with zipfile.ZipFile(source, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in files: z.write(p, 'go2-navigation/' + p.relative_to(ROOT).as_posix())
        z.writestr('go2-navigation/SOURCE_MANIFEST.json', json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    pdf = OUT / 'pdf/navigation_technical_report.zh-CN.pdf'
    if not pdf.is_file(): raise FileNotFoundError(pdf)
    documentation = OUT / 'navigation_documentation.zh-CN.zip'
    with zipfile.ZipFile(documentation, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in files:
            rel = p.relative_to(ROOT)
            if p.suffix == '.md' or rel.parts[0] == 'docs': z.write(p, rel.as_posix())
        if not (ROOT / 'docs/pdf' / pdf.name).is_file(): z.write(pdf, 'pdf/' + pdf.name)
    runtime = OUT / 'go2-navigation-runtime-install-ubuntu24.04-jazzy-harmonic-x86_64.tar.gz'
    install = ROOT / 'install'
    symlinks = []
    with tarfile.open(runtime, 'w:gz', compresslevel=6) as t:
        for p in sorted(install.rglob('*')):
            if '.debug' in p.relative_to(install).parts or p.is_dir(): continue
            if p.is_symlink():
                target = os.readlink(p)
                if os.path.isabs(target) or not p.exists() or not p.resolve().is_relative_to(install):
                    raise RuntimeError('Unsafe or broken install symlink: ' + str(p))
                symlinks.append({'path': str(p.relative_to(ROOT)), 'target': target})
            t.add(p, 'go2-navigation-runtime/' + p.relative_to(ROOT).as_posix(), recursive=False)
        t.add(ROOT / 'docs/编译产物使用.md', 'go2-navigation-runtime/README.zh-CN.md')
        t.add(ROOT / 'docs/build_environment.tsv', 'go2-navigation-runtime/build_environment.tsv')
    with zipfile.ZipFile(source) as z:
        if z.testzip() is not None: raise RuntimeError('Source ZIP CRC failure')
        members = set(z.namelist())
        assert all('go2-navigation/' + n in members for n in REQUIRED)
        assert not any(any(x in OMIT for x in Path(n).parts[1:-1]) for n in members)
    with zipfile.ZipFile(documentation) as z:
        if z.testzip() is not None: raise RuntimeError('Documentation ZIP CRC failure')
    with tarfile.open(runtime) as t:
        assert len(t.getmembers()) > 100
        for m in t:
            if m.isfile():
                f = t.extractfile(m)
                while f.read(1024 * 1024): pass
    summary = {
        'archives': [{'path': str(p.relative_to(ROOT)), 'bytes': p.stat().st_size, 'sha256': digest(p)} for p in (source, runtime, documentation, pdf)],
        'source_files': len(files), 'required_source_assets_present': REQUIRED,
        'source_symlinks': 0, 'runtime_relative_symlinks': symlinks,
        'old_local_build_install_runs_and_implementation_excluded': True,
        'private_host_reference_and_secret_marker_check_passed': True,
        'upstream_examples_only': 'Livox optional LVX sample defaults and one commented image path retain upstream placeholder home paths; no original host username/path or runtime dependency',
        'zip_crc_and_tar_full_read_passed': True,
        'packaging_performs_public_push': False,
        'runtime_abi': 'Linux x86-64 Ubuntu24.04 ROS2 Jazzy Gazebo Harmonic; public dependencies required',
    }
    (OUT / 'delivery_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
