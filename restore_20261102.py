"""Restore and verify the complete 2026-11-02 CAD snapshot. Python >=3.9."""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import shutil
import tempfile
import zipfile

BASE = Path(__file__).resolve().parent

def digest(stream):
    h = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        h.update(block)
    return h.hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((BASE / 'snapshot_20261102_manifest.json').read_text('utf-8'))
    destination = BASE / 'restored_20261102'
    if not args.verify_only and destination.exists():
        raise SystemExit('restored_20261102 already exists; use a clean directory or --verify-only.')
    with tempfile.TemporaryFile() as combined:
        for part in manifest['parts']:
            p = BASE / part['file']
            if p.name != part['file'] or p.stat().st_size != part['bytes']:
                raise ValueError('Invalid part: ' + part['file'])
            with p.open('rb') as stream:
                if digest(stream) != part['sha256']:
                    raise ValueError('Part hash mismatch: ' + part['file'])
                stream.seek(0)
                shutil.copyfileobj(stream, combined)
        if combined.tell() != manifest['archive_bytes']:
            raise ValueError('Combined ZIP length mismatch')
        combined.seek(0)
        if digest(combined) != manifest['archive_sha256']:
            raise ValueError('Combined ZIP hash mismatch')
        combined.seek(0)
        with zipfile.ZipFile(combined) as archive:
            expected = {f['path']: f for f in manifest['files']}
            if set(archive.namelist()) != set(expected) or len(archive.namelist()) != len(expected):
                raise ValueError('ZIP inventory mismatch')
            for name, record in expected.items():
                rel = PurePosixPath(name)
                if rel.is_absolute() or '..' in rel.parts or '\\' in name or ':' in name:
                    raise ValueError('Unsafe ZIP path: ' + name)
                with archive.open(name) as stream:
                    if archive.getinfo(name).file_size != record['bytes'] or digest(stream) != record['sha256']:
                        raise ValueError('File hash mismatch: ' + name)
            if not args.verify_only:
                destination.mkdir()
                for name in expected:
                    target = destination.joinpath(*PurePosixPath(name).parts)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with archive.open(name) as source, target.open('xb') as output:
                        shutil.copyfileobj(source, output)
    print('Verified %d files; %s' % (len(manifest['files']), 'no extraction requested' if args.verify_only else destination))

if __name__ == '__main__':
    main()
