#!/usr/bin/env python3
"""Prepare immutable run-only FastDDS presets. No ROS, participant or process starts."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
NS={'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def validate_xml(path,capacity):
    root=ET.parse(path).getroot()
    descriptors=root.findall('d:profiles/d:transport_descriptors/d:transport_descriptor',NS)
    if len(descriptors)!=2:raise ValueError('Exactly one UDP and one SHM transport required')
    by_type={d.findtext('d:type',namespaces=NS):d for d in descriptors}
    udp=by_type['UDPv4'];shm=by_type['SHM']
    if [x.text for x in udp.findall('d:interfaceWhiteList/d:address',NS)]!=['127.0.0.1']:raise ValueError('UDP must use loopback only')
    for node,key,value in [(udp,'maxMessageSize','65500'),(udp,'maxInitialPeersRange','32'),(udp,'sendBufferSize','0'),(udp,'receiveBufferSize','0'),(shm,'segment_size',str(capacity)),(shm,'maxMessageSize','65500'),(shm,'port_queue_capacity','512')]:
        if node.findtext('d:'+key,namespaces=NS)!=value:raise ValueError('Unexpected transport setting '+key)
    participants=root.findall('d:profiles/d:participant',NS)
    if len(participants)!=1 or participants[0].get('is_default_profile')!='true':raise ValueError('Exactly one default profile required')
    rtps=participants[0].find('d:rtps',NS)
    if rtps.findtext('d:useBuiltinTransports',namespaces=NS)!='false':raise ValueError('Builtin transport would contaminate the A/B')
    if [p.text for p in rtps.findall('d:userTransports/d:transport_id',NS)]!=['audit_udp_loopback','audit_shm']:raise ValueError('User transports differ')
    for key in ['metatrafficUnicastLocatorList','initialPeersList']:
        locators=rtps.findall('d:builtin/d:'+key+'/d:locator',NS)
        if len(locators)!=1 or locators[0].findtext('d:udpv4/d:address',namespaces=NS)!='127.0.0.1' or locators[0].findtext('d:udpv4/d:port',namespaces=NS)!='0':raise ValueError('Discovery must be explicit loopback')
    if rtps.findall('d:builtin/d:metatrafficMulticastLocatorList/d:locator',NS):raise ValueError('No multicast locator permitted')
    if root.findall('d:profiles/d:publisher',NS)or root.findall('d:profiles/d:subscriber',NS):raise ValueError('This experiment does not override endpoint QoS')
    return True

def prepare(run,profile):
    run=Path(run).resolve()
    if not run.is_dir():raise ValueError('Run directory must already exist')
    if any((run/p).exists()for p in ['cloud_transport.xml','cloud_transport_manifest.json','worker_ready','telemetry.jsonl','actuator.jsonl']):raise RuntimeError('Refuse overwrite or live run')
    config_path=HERE/'config_env_manifest.json';config=json.loads(config_path.read_text())
    selected=config['profiles'][profile];original=HERE/selected['filename']
    if sha(original)!=selected['sha256']:raise RuntimeError('Frozen XML mismatch')
    for path,digest in config['installed_binary_sha256'].items():
        if sha(path)!=digest:raise RuntimeError('DDS/RMW binary differs from audited version '+path)
    validate_xml(original,selected['segment_capacity_bytes'])
    copied=run/'cloud_transport.xml'
    with copied.open('xb')as stream:stream.write(original.read_bytes())
    environment={**config['common_environment'],config['runtime_environment_path_key']:str(copied)}
    refs={str(path.resolve()):sha(path)for path in [Path(__file__),config_path,original]}
    refs.update(config['installed_binary_sha256'])
    refs.update(config['source_evidence'])
    receipt={'schema':'cloud_transport_experiment/v1','run':str(run),'profile':profile,'xml_path':str(copied),'xml_sha256':sha(copied),
        'environment':environment,'remove_environment_keys':config['remove_environment_keys'],'source_hashes':refs,
        'segment_capacity_bytes':selected['segment_capacity_bytes'],'expected_transport':config['expected_transport'],
        'environment_scope':'Only all children owned by this run and the independently launched probe; never process-global or other jobs',
        'comparison':config['comparison'],'publisher_QoS_not_overridden':True,'source_header_timestamp_unchanged':True,'command_TTL_s_unchanged':.3,
        'kernel_parameters_changed':False,'configuration_only':True,'actual_transport_verified':False,'ROS_started':False,'participant_created':False}
    with(run/'cloud_transport_manifest.json').open('x')as stream:stream.write(json.dumps(receipt,indent=2)+'\n')
    return receipt

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--profile',choices=['shm_512k','shm_64m'],required=True);a=p.parse_args()
    r=prepare(a.run,a.profile);print(json.dumps({'profile':r['profile'],'manifest':str(a.run/'cloud_transport_manifest.json'),'xml_sha256':r['xml_sha256'],'environment':r['environment']}))
if __name__=='__main__':main()
