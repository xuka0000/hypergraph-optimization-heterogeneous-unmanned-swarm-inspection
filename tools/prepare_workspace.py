"""Restore data and current source into a NEW separate directory; launch no solver."""
from pathlib import Path
import argparse, datetime, hashlib, json, shutil, sys, zipfile
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'book/reproduction'))
import package_tools as frozen
sys.path.insert(0,str(ROOT/'tools'))
from verify_package import sha

def main():
    p=argparse.ArgumentParser();p.add_argument('--destination',type=Path,required=True);a=p.parse_args()
    dest=a.destination.resolve()
    if dest.exists() or dest.is_relative_to(ROOT) or ROOT.is_relative_to(dest):
        p.error('Use a nonexistent directory outside this repository.')
    manifest=json.loads((ROOT/'MANIFEST.json').read_text(encoding='utf8'))
    for e in manifest['files']:
        if sha(ROOT/e['path'])!=e['sha256']:raise ValueError('Changed package file: '+e['path'])
    extra=json.loads((ROOT/'data/archives.json').read_text(encoding='utf8'))['archives']
    old=frozen.read(ROOT/'book/reproduction/archive_inventory.json')['archives']
    size=sum(x['uncompressed_bytes'] for x in old)+sum(e['bytes'] for z in extra for e in z['files'])+128*2**20
    parent=dest.parent
    while not parent.exists():parent=parent.parent
    floor=(10 if parent.drive.upper()=='C:' else 30)*2**30
    if shutil.disk_usage(parent).free<size+floor:raise ValueError('Insufficient disk space for extraction and safety floor.')
    changes=[]
    frozen.extract(dest,'frozen',True)
    for pth in sorted((ROOT/'project').rglob('*')):
        if not pth.is_file():continue
        out=dest/pth.relative_to(ROOT/'project');out.parent.mkdir(parents=True,exist_ok=True)
        if out.exists() and sha(out)!=sha(pth):changes.append(dict(path=out.relative_to(dest).as_posix(),before=sha(out),after=sha(pth),kind='current_source_overlay'))
        shutil.copyfile(pth,out)
    for arc in extra:
        archive=ROOT/arc['path']
        if sha(archive)!=arc['sha256']:raise ValueError('Archive hash mismatch')
        with zipfile.ZipFile(archive) as z:
            for e in arc['files']:
                out=frozen.inside(dest,e['path']);out.parent.mkdir(parents=True,exist_ok=True)
                if out.exists() and sha(out)==e['sha256']:continue
                if out.exists():changes.append(dict(path=e['path'],before=sha(out),after=e['sha256'],kind='current_data_overlay'))
                with z.open(e['path']) as src,out.open('wb') as dst:shutil.copyfileobj(src,dst)
                if sha(out)!=e['sha256']:raise ValueError('Extracted member mismatch: '+e['path'])
    receipt=dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_snapshot='CURRENT_SOURCES.json',overlay_changes=changes,experiments_started=0,historical_archives_unchanged=True)
    (dest/'RESTORE_RECEIPT.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(destination=str(dest),overlay_changes=len(changes),experiments_started=0),ensure_ascii=False))
if __name__=='__main__':main()
