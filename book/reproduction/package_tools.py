"""Inspect frozen material or unpack it into a NEW, separate directory.

No experiment, optimizer, cloud request, or original workspace is invoked.
Python 3 standard library only. Run --help for the available operations.
"""
import argparse
import hashlib
import json
import shutil
import sys
import zipfile
from pathlib import Path

BOOK = Path(__file__).resolve().parents[1]
PACKAGE = BOOK / 'reproduction'
VERSIONS = {
    'frozen': None,
    'regional-c943': 'regional_resource_source_bindings.json',
    'regional-2bf': 'latest_regional_source_bindings.json',
    'conditional-613b': 'conditional_source_bindings.json',
}


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inside(root, relative):
    # Apply this to both ZIP names and manifest paths before filesystem access.
    target = (root / relative.replace('\\', '/')).resolve()
    if target == root.resolve() or not target.is_relative_to(root.resolve()):
        raise ValueError(f'Unsafe relative path: {relative}')
    return target


def verify(deep=False):
    errors = []
    checked = 0
    for entry in read(PACKAGE / 'source_manifest.json')['files']:
        path = inside(PACKAGE / 'project', entry['path'])
        checked += 1
        if not path.is_file() or sha(path) != entry['sha256']:
            errors.append(f"source: {entry['path']}")
    missing = {}
    for name in [v for v in VERSIONS.values() if v]:
        manifest = read(PACKAGE / name)
        missing[name] = manifest.get('missing', [])
        for entry in manifest['files']:
            path = inside(BOOK, entry['electronic_path'])
            checked += 1
            if not path.is_file() or sha(path) != entry['sha256']:
                errors.append(f"version: {name}: {entry['path']}")
    for archive in read(PACKAGE / 'archive_inventory.json')['archives']:
        path = inside(PACKAGE, archive['path'])
        if not path.is_file() or sha(path) != archive['sha256']:
            errors.append(f"archive: {archive['path']}")
            continue
        if deep:
            with zipfile.ZipFile(path) as bundle:
                expected = {f['path']: f for f in archive['files']}
                names = bundle.namelist()
                if len(names) != len(set(names)) or set(names) != set(expected):
                    errors.append(f"archive member list: {archive['path']}")
                for item in bundle.infolist():
                    inside(PACKAGE, item.filename)
                    digest = hashlib.sha256()
                    with bundle.open(item) as stream:
                        for block in iter(lambda: stream.read(1024 * 1024), b''):
                            digest.update(block)
                    checked += 1
                    if item.filename not in expected or digest.hexdigest() != expected[item.filename]['sha256']:
                        errors.append(f"archive member: {archive['path']}: {item.filename}")
    return {'files_checked': checked, 'errors': errors,
            'known_unrecovered_historical_sources': missing,
            'meaning': 'Byte integrity only; no claim of numerical reproducibility or physical validation.'}


def extract(destination, version, include_data):
    destination = destination.resolve()
    if destination.exists():
        raise ValueError('Destination must not exist; existing projects are never overwritten.')
    if destination.is_relative_to(BOOK) or BOOK.is_relative_to(destination):
        raise ValueError('Use a separate disposable reproduction directory outside the book.')
    bindings = read(PACKAGE / VERSIONS[version]) if VERSIONS[version] else {'files': [], 'missing': []}
    if bindings.get('missing'):
        raise ValueError('This historical version is not byte-complete. Read its missing-source record; extraction as an exact version is refused.')
    check = verify()
    if check['errors']:
        raise ValueError(f"Package integrity failed: {check['errors']}")
    ancestor = destination.parent
    while not ancestor.exists():
        ancestor = ancestor.parent
    inventory = read(PACKAGE / 'archive_inventory.json')['archives'] if include_data else []
    required = sum(x['bytes'] for x in read(PACKAGE / 'source_manifest.json')['files'])
    required += sum(a['uncompressed_bytes'] for a in inventory)
    required += sum(inside(BOOK, x['electronic_path']).stat().st_size for x in bindings['files'])
    floor = 10 * 2**30 if ancestor.drive.upper() == 'C:' else 30 * 2**30
    if shutil.disk_usage(ancestor).free < floor + required:
        raise ValueError('Insufficient free space after counting peak extraction writes and the local storage floor.')
    shutil.copytree(PACKAGE / 'project', destination)
    extracted = {}
    for archive in inventory:
        with zipfile.ZipFile(PACKAGE / archive['path']) as bundle:
            for entry in archive['files']:
                path = inside(destination, entry['path'])
                if path.exists():
                    if sha(path) != entry['sha256']:
                        raise ValueError(f"Conflicting archived versions: {entry['path']}; partial extraction retained for inspection.")
                    continue
                path.parent.mkdir(parents=True, exist_ok=True)
                with bundle.open(entry['path']) as source, path.open('xb') as output:
                    shutil.copyfileobj(source, output)
                if sha(path) != entry['sha256']:
                    raise ValueError(f"Extracted hash mismatch: {entry['path']}")
                extracted[entry['path']] = entry['sha256']
    for entry in bindings['files']:
        path = inside(destination, entry['path'])
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(inside(BOOK, entry['electronic_path']), path)
        assert sha(path) == entry['sha256']
    receipt = {'version': version, 'source_binding': VERSIONS[version],
               'data_files_extracted': len(extracted), 'experiments_started': 0,
               'warning': 'Inspect original entry-point roots, output paths, dependencies and resource guards before any execution. No performance reproduction has been performed.'}
    (destination / 'BOOK_EXTRACTION_RECEIPT.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2), encoding='utf-8')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deep', action='store_true', help='Verify each compressed member without extracting it.')
    parser.add_argument('--extract', type=Path, help='Create a new, separate reproduction tree; never run it.')
    parser.add_argument('--version', choices=VERSIONS, default='frozen')
    parser.add_argument('--include-data', action='store_true')
    args = parser.parse_args()
    report = extract(args.extract, args.version, args.include_data) if args.extract else verify(args.deep)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    sys.exit(1 if report.get('errors') else 0)
