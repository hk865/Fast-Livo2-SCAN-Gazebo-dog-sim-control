"""Frozen private DDS process environment; no DDS imports or message-path work."""
import hashlib
import json
import os
from pathlib import Path
import re

HERE=Path(__file__).resolve().parent
# Source-controlled locator contains no host artifact pins. Actual identities
# are generated locally; an explicit environment selection is process-fixed.
_locator=json.loads((HERE/'PRIVATE_DDS_CONTRACT.json').read_text())
if (set(_locator)!={'schema','local_contract'} or
        _locator['schema']!='private_dds_contract_locator/v1'):
    raise RuntimeError('Exact private DDS local contract locator required')
CONTRACT=Path(os.environ.get('PRIVATE_DDS_RUNTIME_CONTRACT',
    str(HERE/_locator['local_contract']))).expanduser().resolve()
DOMAIN_QUERY_DIRECTORY=CONTRACT.parent/'domain_query_witness'
ENV_KEYS=('LD_PRELOAD','PRIVATE_DDS_WITNESS_DESCRIPTOR','PRIVATE_DDS_WITNESS_DIR',
          'PRIVATE_DDS_RUNTIME_CONTRACT','PRIVATE_DDS_RUNTIME_CONTRACT_SHA256','OWNER_WAIT_PREFIX')
_verified=None


def _hash(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def _sha(value):
    return type(value)is str and len(value)==64 and all(c in '0123456789abcdef'for c in value)


def load_contract():
    """Hash frozen inputs once/process before launching any owned DDS child."""
    global _verified
    try:raw=CONTRACT.read_bytes()
    except FileNotFoundError as error:
        raise RuntimeError('Private DDS binding missing; follow docs/PRIVATE_DDS.md to generate .local/private_dds/PRIVATE_DDS_CONTRACT.json') from error
    if len(raw)>65536:raise RuntimeError('Private DDS contract exceeds bound')
    checksum=hashlib.sha256(raw).hexdigest()
    if _verified is not None:
        if _verified['contract_sha256']!=checksum:raise RuntimeError('Private DDS contract changed after verification')
        return _verified
    value=json.loads(raw)
    if (type(value)is not dict or set(value)!={'schema','library','participant_witness','descriptor','owner_probe'}
            or value['schema']!='private_dds_runtime/v1'):
        raise RuntimeError('Exact private DDS contract schema required')
    artifacts={}
    for role in ('library','participant_witness','descriptor','owner_probe'):
        item=value[role];keys={'path','sha256','build_id'}if role=='library'else{'path','sha256'}
        if type(item)is not dict or set(item)!=keys or not _sha(item.get('sha256')):
            raise RuntimeError('Malformed private DDS artifact '+role)
        path=Path(item['path'])
        if not path.is_absolute() or not path.is_file() or str(path.resolve())!=str(path):
            raise RuntimeError('Canonical absolute existing private DDS artifact required: '+role)
        if _hash(path)!=item['sha256']:raise RuntimeError('Private DDS artifact SHA differs: '+role)
        artifacts[role]=dict(item)
    library=artifacts['library'];build_id=library['build_id']
    if (type(build_id)is not str or not 0<len(build_id)<=128 or len(build_id)%2
            or any(c not in '0123456789abcdef'for c in build_id)
            or any(Path(library['path']).is_relative_to(p) for p in ('/usr','/lib','/lib64','/opt/ros'))):
        raise RuntimeError('Private DDS build ID invalid or a system/ROS DDS was selected')
    text=Path(artifacts['descriptor']['path']).read_text()
    if len(text)>8192:raise RuntimeError('Private DDS descriptor exceeds bound')
    fields={}
    for line in text.splitlines():
        if not line or line.startswith('#'):continue
        key,separator,val=line.partition('=')
        if not separator or key in fields:raise RuntimeError('Private DDS descriptor malformed/duplicate')
        fields[key]=val
    if fields!={'schema':'private_fastdds_descriptor/v1','library':library['path'],
                'sha256':library['sha256'],'build_id':build_id}:
        raise RuntimeError('Private DDS descriptor does not bind frozen library')
    _verified=dict(contract_path=str(CONTRACT),contract_sha256=checksum,artifacts=artifacts)
    return _verified


def binding():
    checked=load_contract()
    return json.loads(json.dumps(checked))


def _preloads(value):
    if type(value)is not str:raise RuntimeError('LD_PRELOAD must be a string')
    return [part for part in re.split(r'[:\s]+',value)if part]


def reject_unknown_preloads(environment,*,allow_owner=False):
    artifacts=load_contract()['artifacts']
    allowed={artifacts['participant_witness']['path']}
    if allow_owner:allowed.add(artifacts['owner_probe']['path'])
    parts=_preloads(environment.get('LD_PRELOAD',''))
    if len(parts)!=len(set(parts)) or any(part not in allowed for part in parts):
        raise RuntimeError('Unknown/duplicate inherited LD_PRELOAD; refusing to discard or carry it')
    return parts


def _directory(path,*,fresh):
    path=Path(path)
    if not path.is_absolute() or path.is_symlink():raise RuntimeError('Absolute owned witness directory required')
    path.mkdir(mode=0o700,parents=True,exist_ok=not fresh)
    if not path.is_dir() or path.stat().st_uid!=os.getuid():raise RuntimeError('Witness directory ownership differs')
    return str(path.resolve())


def configure(environment,*,witness_directory=DOMAIN_QUERY_DIRECTORY,fresh=False):
    checked=load_contract();reject_unknown_preloads(environment)
    result=dict(environment);directory=_directory(witness_directory,fresh=fresh)
    prefix=str(Path(checked['artifacts']['library']['path']).parent)
    paths=[p for p in result.get('LD_LIBRARY_PATH','').split(':')if p and p!=prefix]
    result['LD_LIBRARY_PATH']=':'.join([prefix,*paths])
    result.update(LD_PRELOAD=checked['artifacts']['participant_witness']['path'],
        PRIVATE_DDS_WITNESS_DESCRIPTOR=checked['artifacts']['descriptor']['path'],
        PRIVATE_DDS_WITNESS_DIR=directory,PRIVATE_DDS_RUNTIME_CONTRACT=checked['contract_path'],
        PRIVATE_DDS_RUNTIME_CONTRACT_SHA256=checked['contract_sha256'])
    verify_environment(result,witness_directory=Path(directory))
    return result


def verify_environment(environment,*,witness_directory,allow_owner=False):
    checked=load_contract();artifacts=checked['artifacts'];parts=reject_unknown_preloads(environment,allow_owner=allow_owner)
    expected=[artifacts['participant_witness']['path']]
    if allow_owner:expected.append(artifacts['owner_probe']['path'])
    if parts!=expected:raise RuntimeError('Private DDS participant/native preload binding overwritten')
    if environment.get('LD_LIBRARY_PATH','').split(':')[0]!=str(Path(artifacts['library']['path']).parent):
        raise RuntimeError('Private DDS library prefix overwritten after setup/transport/diagnostic merge')
    values={'PRIVATE_DDS_WITNESS_DESCRIPTOR':artifacts['descriptor']['path'],
        'PRIVATE_DDS_WITNESS_DIR':str(Path(witness_directory).resolve()),
        'PRIVATE_DDS_RUNTIME_CONTRACT':checked['contract_path'],
        'PRIVATE_DDS_RUNTIME_CONTRACT_SHA256':checked['contract_sha256']}
    if any(environment.get(k)!=v for k,v in values.items()):raise RuntimeError('Private DDS protected witness environment overwritten')
    if not allow_owner and environment.get('OWNER_WAIT_PREFIX'):
        raise RuntimeError('Owner probe prefix must not be global')
    return True


def native_environment(run,environment):
    run=Path(run).resolve();directory=run/'private_dds_participants'
    verify_environment(environment,witness_directory=directory)
    owner_directory=run/'owner_wait';owner_directory.mkdir(mode=0o700,exist_ok=False)
    checked=load_contract();extra={'DEMO_RUN_DIR':str(run),
        'LD_PRELOAD':':'.join([checked['artifacts']['participant_witness']['path'],checked['artifacts']['owner_probe']['path']]),
        'OWNER_WAIT_PREFIX':str(owner_directory/'probe')}
    verify_environment({**environment,**extra},witness_directory=directory,allow_owner=True)
    return extra


def pure_environment_checks():
    """No filesystem writes or child processes; requires frozen real contract."""
    a=load_contract()['artifacts'];prefix=str(Path(a['library']['path']).parent);directory=DOMAIN_QUERY_DIRECTORY.resolve()
    base={'LD_LIBRARY_PATH':prefix+':/opt/ros/jazzy/lib','LD_PRELOAD':a['participant_witness']['path'],
        'PRIVATE_DDS_WITNESS_DESCRIPTOR':a['descriptor']['path'],'PRIVATE_DDS_WITNESS_DIR':str(directory),
        'PRIVATE_DDS_RUNTIME_CONTRACT':str(CONTRACT),'PRIVATE_DDS_RUNTIME_CONTRACT_SHA256':load_contract()['contract_sha256']}
    assert verify_environment(base,witness_directory=directory)
    rejected=0
    for key,value in [('LD_PRELOAD','/unknown.so'),('LD_PRELOAD',a['owner_probe']['path']),
                      ('LD_LIBRARY_PATH','/opt/ros/jazzy/lib:'+prefix),('PRIVATE_DDS_WITNESS_DIR','/tmp/wrong'),
                      ('PRIVATE_DDS_WITNESS_DESCRIPTOR','/tmp/wrong'),('OWNER_WAIT_PREFIX','/tmp/global')]:
        try:verify_environment({**base,key:value},witness_directory=directory)
        except RuntimeError:rejected+=1
        else:raise AssertionError('Unsafe environment accepted: '+key)
    return dict(passed=True,rejected=rejected,files_written=False,children_started=False)
