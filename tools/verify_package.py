"""Verify delivery bytes; optionally verify each historical ZIP member."""
from pathlib import Path
import argparse, hashlib, json, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--deep',action='store_true');a=p.parse_args()
    data=json.loads((ROOT/'MANIFEST.json').read_text(encoding='utf8'));errors=[]
    for e in data['files']:
        f=(ROOT/e['path']).resolve()
        if not f.is_relative_to(ROOT) or not f.is_file() or sha(f)!=e['sha256']:errors.append(e['path'])
    print(json.dumps(dict(files_checked=len(data['files']),errors=errors),ensure_ascii=False))
    if errors:return 1
    if a.deep:
        return subprocess.call([sys.executable,'-B',str(ROOT/'book/reproduction/package_tools.py'),'--deep'],cwd=ROOT)
    return 0
if __name__=='__main__':raise SystemExit(main())
