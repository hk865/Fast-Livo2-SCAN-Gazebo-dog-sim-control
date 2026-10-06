#!/usr/bin/env python3
"""Compare complete captured CDR identity only; not a frontend replay PASS."""
import argparse
import json
from pathlib import Path
from input_identity import compare, load_index, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left', type=Path, required=True)
    p.add_argument('--right', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    result = compare(load_index(args.left), load_index(args.right))
    result['input_sha256'] = {str(args.left.resolve()): sha(args.left), str(args.right.resolve()): sha(args.right)}
    with args.out.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps({'status': result['status'], 'out': str(args.out)}))
    return 0 if result['status'] == 'passed_captured_identity_only' else 1


if __name__ == '__main__':
    raise SystemExit(main())
