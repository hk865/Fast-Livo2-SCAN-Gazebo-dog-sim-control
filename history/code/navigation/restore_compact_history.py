"""Restore original JSON objects; explicit prefix mode never certifies completion."""
import argparse
import hashlib
import json
from pathlib import Path
from compact_archives import decode, context


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--allow-prefix', action='store_true')
    parser.add_argument('--max-output-bytes', type=int, default=64*1024**2)
    parser.add_argument('--receipt', type=Path, required=True)
    args = parser.parse_args()
    if args.max_output_bytes <= 0: raise ValueError('Positive output limit required')
    paths = [args.input.resolve(), args.output.resolve(), args.receipt.resolve()]
    if len(set(paths)) != 3:
        raise ValueError('Input, output and receipt must be distinct paths')
    protected = {args.input.parent.joinpath(name).resolve()
        for name in ('input_profile.json', 'source_manifest.json')}
    if paths[1] in protected or paths[2] in protected:
        raise ValueError('Output/receipt cannot replace source identity files')
    for i, first in enumerate(paths):
        for second in paths[i+1:]:
            if first.exists() and second.exists() and first.samefile(second):
                raise ValueError('Input/output/receipt inode alias refused')
    report = dict(input=str(args.input), output=str(args.output), output_bytes=0,
        output_records_written=0, raw_lexical_whitespace_preserved=False,
        original_JSON_values_preserved=True, formal_acceptance_allowed=False)
    checksum = hashlib.sha256()
    # Reserve a fresh receipt first; no existing evidence is overwritten even on failure.
    with args.receipt.open('x') as receipt_stream:
        try:
            sources = context(args.input.parent)
            with args.input.open() as source, args.output.open('x') as target:
                for row in decode(source, require_complete=not args.allow_prefix,
                        expected_sources=sources,
                        expected_original_filename=args.input.name.replace('.compact.jsonl', '.jsonl'),
                        integrity=report):
                    line = json.dumps(row, allow_nan=False, ensure_ascii=False)+'\n'
                    raw = line.encode('utf-8')
                    if report['output_bytes'] + len(raw) > args.max_output_bytes:
                        raise RuntimeError('Restored output limit exceeded; partial output retained')
                    target.write(line); checksum.update(raw); report['output_bytes'] += len(raw)
                    report['output_records_written'] += 1
            report['output_sha256'] = checksum.hexdigest()
        except Exception as error:
            report.update(error=type(error).__name__+': '+str(error), restore_completed=False)
            raise
        else: report['restore_completed'] = True
        finally: receipt_stream.write(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__': main()
