"""Retrieve explicitly excluded upstream source files into a restored workspace.

Downloads exact documented bytes over HTTPS; verifies SHA256 before installing.
It does not redistribute the files through this repository or execute them.
"""
from pathlib import Path
import argparse, hashlib, json, urllib.request
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--list-only',action='store_true');a=p.parse_args()
work=a.workspace.resolve()
if not work.is_dir():p.error('First run prepare_workspace.py.')
source=work/'总结/03_实验数据/电池能耗公开机型验证/source_manifest.json'
records=json.loads(source.read_text(encoding='utf-8-sig'))['files']
by_path={x['path'].replace('\\','/'):x for x in records if x.get('url')}
excluded=json.loads((ROOT/'data/external_assets.json').read_text(encoding='utf8'))['not_redistributed']
missing=[x['path'] for x in excluded if x['path'] not in by_path]
if missing:raise SystemExit('No documented source URL for: '+str(missing))
if a.list_only:
    print(json.dumps(dict(external_files=len(excluded),documented_urls=len(excluded),downloads=0)))
else:
    done=0
    for e in excluded:
        target=(work/e['path']).resolve()
        if not target.is_relative_to(work):raise ValueError('Unsafe target')
        if target.exists():
            if hashlib.sha256(target.read_bytes()).hexdigest()!=e['sha256']:raise ValueError('Existing different file: '+e['path'])
            continue
        url=by_path[e['path']]['url']
        if not url.startswith('https://'):raise ValueError('Only documented HTTPS URLs are accepted')
        request=urllib.request.Request(url,headers={'User-Agent':'inspection-study-asset-restore'})
        with urllib.request.urlopen(request,timeout=60) as response:data=response.read()
        if hashlib.sha256(data).hexdigest()!=e['sha256']:raise ValueError('Downloaded hash mismatch: '+e['path'])
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data);done+=1
    print(json.dumps(dict(downloaded_verified_files=done,executed_files=0)))
