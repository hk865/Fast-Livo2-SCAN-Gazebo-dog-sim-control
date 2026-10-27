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

def prepare(run,profile=None):
    run=Path(run).resolve()
    if any((run/p).exists()for p in ['cloud_transport.xml','cloud_transport_manifest.json','worker_ready']):raise RuntimeError('Refuse existing/live output')
    root=Path(__file__).resolve().parents[1]
    settings=json.loads((root/'config/navigation.json').read_text())['dds_transport']
    original=root/'config/dds_transport.xml'
    validate_xml(original,settings['segment_capacity_bytes'])
    copied=run/'cloud_transport.xml'
    with copied.open('xb')as f:f.write(original.read_bytes())
    environment={**settings['environment'],settings['runtime_environment_path_key']:str(copied)}
    receipt=dict(run=str(run),xml_path=str(copied),environment=environment,remove_environment_keys=settings['remove_environment_keys'],source_hashes={},segment_capacity_bytes=settings['segment_capacity_bytes'],source_header_timestamp_unchanged=True,command_TTL_s_unchanged=.3,ROS_started=False)
    (run/'cloud_transport_manifest.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    print(json.dumps(prepare(a.run)))
if __name__=='__main__':main()
