#!/usr/bin/env python3
"""Generate local exact-byte DDS contract/descriptor. Never selects system DDS.

Use --bundle for newly built observers, or --from-contract to rebind an existing
reviewed artifact tuple. Rebinding verifies its hashes/descriptor, not a new build.
No library, DDS participant, ROS or navigation process is loaded or executed.
"""
import argparse
from pathlib import Path
import sys
sys.dont_write_bytecode = True
from common import ROOT, artifact, elf_identity, private_path, read, run, sha, write_new


def checked(item):
    path = Path(item['path'])
    if not path.is_absolute() or str(path.resolve(strict=True)) != str(path) or sha(path) != item['sha256']:
        raise ValueError('Artifact path/SHA mismatch: '+str(path))
    return dict(item)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--bundle', type=Path, help='OBSERVER_BUILD.json from build_observers.py')
    choice.add_argument('--from-contract', type=Path, help='Previously independently validated v1 contract; all exact pins must still match')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'.local/private_dds')
    args = parser.parse_args()
    source = read(args.bundle or args.from_contract)
    if args.bundle:
        if source.get('schema') != 'private_dds_observer_build/v1':
            raise ValueError('Unknown observer bundle')
        library = checked(source['library'])
        witness = checked(source['participant_witness'])
        probe = checked(source['owner_probe'])
    else:
        if source.get('schema') != 'private_dds_runtime/v1' or set(source) != {'schema','library','participant_witness','descriptor','owner_probe'}:
            raise ValueError('Expected complete reviewed private DDS runtime contract')
        library, witness, probe = [checked(source[k]) for k in ('library','participant_witness','owner_probe')]
        descriptor = checked(source['descriptor'])
        expected = dict(schema='private_fastdds_descriptor/v1', library=library['path'],
                        sha256=library['sha256'], build_id=library['build_id'])
        lines = Path(descriptor['path']).read_text().splitlines()
        fields = {}
        for line in lines:
            if not line or line.startswith('#'):
                continue
            key, separator, value = line.partition('=')
            if not separator or key in fields:
                raise ValueError('Malformed/duplicate descriptor key')
            fields[key] = value
        if fields != expected:
            raise ValueError('Previously reviewed descriptor does not bind library')
    if elf_identity(library['path']) != library:
        raise ValueError('Exact library ELF identity differs')
    dynamic = run(['readelf','-d',witness['path']])
    if any(x in dynamic for x in ['[libfastrtps','[libfastdds']):
        raise ValueError('Creation witness has an unexpected DDS dependency')
    raw = Path(probe['path']).read_bytes()
    if library['path'].encode() not in raw or bytes.fromhex(library['build_id']) not in raw:
        raise ValueError('Probe does not guard this canonical private library path/BuildID')
    output = private_path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    descriptor_path, contract_path = output/'participant.descriptor', output/'PRIVATE_DDS_CONTRACT.json'
    if descriptor_path.exists() or contract_path.exists():
        raise ValueError('Local descriptor/contract already exists; use a new output directory')
    with descriptor_path.open('x') as stream:
        stream.write('schema=private_fastdds_descriptor/v1\nlibrary='+library['path']+
                     '\nsha256='+library['sha256']+'\nbuild_id='+library['build_id']+'\n')
    contract = dict(schema='private_dds_runtime/v1', library=library,
                    participant_witness=witness, descriptor=artifact(descriptor_path), owner_probe=probe)
    write_new(contract_path, contract)
    write_new(output/'BINDING_RECEIPT.json', dict(schema='private_dds_local_binding/v1',
        source=artifact(args.bundle or args.from_contract), contract=artifact(contract_path),
        mode='new_build' if args.bundle else 'reuse_previously_reviewed_tuple',
        actual_participant_loading_verified=False, navigation_PASS=False))
    print(contract_path)


if __name__ == '__main__':
    main()
