"""Extract only preselected complete author members after whole-archive MD5 check."""
from pathlib import Path
import tarfile,json,hashlib,time,datetime
from run_hypergraph_scip_demo import ROOT,load,save,sha

def main():
    base=ROOT/'总结/07_证据与复现索引/超图建模与原生求解/公开数据'
    receipt=load(base/'jgaa2026_full_download_receipt.json');assert receipt['status']=='complete'
    keys=['ibm01','ibm02','us04','bibd_49_3','AProVE07-01.cnf.dual','AProVE07-27.cnf.dual','superblue19']
    dest=base/'JGAA2026';dest.mkdir(exist_ok=True);catalog=[];selected=[];started=time.monotonic()
    with tarfile.open(base/'hypergraph-data.tar.gz','r|gz') as tar:
        for m in tar:
            assert time.monotonic()-started<600,'Finite extraction guard'
            if not m.isfile():continue
            row={'name':m.name,'bytes':m.size}
            if m.name.endswith(('.hgr','.hgr.random100')):
                f=tar.extractfile(m);first=f.readline();row['header']=first.decode().strip()
                if any(k.lower() in m.name.lower() for k in keys):
                    assert m.size<100*1024*1024
                    p=dest/Path(m.name).name;assert not p.exists()
                    with p.open('wb') as out:
                        out.write(first)
                        for b in iter(lambda:f.read(1024*1024),b''):out.write(b)
                    assert p.stat().st_size==m.size
                    row.update(path=p.relative_to(ROOT).as_posix(),sha256=sha(p));selected.append(row);print(json.dumps(row),flush=True)
            catalog.append(row)
    save(base/'jgaa2026_complete_catalog.json',catalog)
    assert len(selected)==7,'All seven frozen complete members required'
    save(dest/'extraction_receipt.json',{'observed_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'archive_md5_verified':receipt['md5'],'archive_sha256':receipt['sha256'],'selection_rule':str(base/'public_selection_frozen.json'),'full_members':selected,'elapsed_seconds':time.monotonic()-started})

if __name__=='__main__':main()
