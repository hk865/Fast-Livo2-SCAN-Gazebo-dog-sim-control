"""Owned64MiB FastDDS configuration, freshly bind local binaries; no old PASS."""
from pathlib import Path
import json
from portable_common import REPO,sha

def prepare(run):
    run=Path(run).resolve();base=REPO/'multifloor_demo/teacher_mode/tests/cloud_transport_probe_20261004'
    config=json.loads((base/'config_env_manifest.json').read_text());selected=config['profiles']['shm_64m'];source=base/selected['filename'];raw=source.read_bytes()
    if sha(source)!=selected['sha256']:raise RuntimeError('Exported64MiB transport XML differs')
    import importlib.util
    spec=importlib.util.spec_from_file_location('portable_XML_only',base/'prepare_transport.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.validate_xml(source,selected['segment_capacity_bytes'])
    binaries={}
    for old in config['installed_binary_sha256']:
        p=Path(old)
        if not p.is_file()or not p.is_relative_to('/opt/ros/jazzy'):raise RuntimeError('Declared ROSJazzy RMW binary unavailable '+str(p))
        binaries[str(p)]=sha(p)
    dest=run/'cloud_transport.xml'
    with dest.open('xb')as f:f.write(raw)
    d={'schema':'portable_owned_transport/v1','status':'LOCAL_BINARY_XML_BINDING_ONLY','run':str(run),'profile':'shm_64m','xml_path':str(dest),'xml_sha256':sha(dest),'environment':{**config['common_environment'],config['runtime_environment_path_key']:str(dest)},'remove_environment_keys':config['remove_environment_keys'],'source_hashes':{str(Path(__file__).resolve()):sha(Path(__file__)),str(source):sha(source),**binaries},'segment_capacity_bytes':selected['segment_capacity_bytes'],'expected_transport':config['expected_transport'],'original_transport_PASS_inherited':False,'actual_transport_delivery_verified':False,'scope':'Only children owned by this new run; actual source delivery and freshness require runtime evidence'}
    with(run/'cloud_transport_manifest.json').open('x')as f:f.write(json.dumps(d,indent=2)+'\n')
    return d
