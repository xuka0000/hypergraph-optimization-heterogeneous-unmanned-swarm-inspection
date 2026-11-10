"""Export every constraint of a saved binary unit-incidence packing instance.

No solver or training runs. Use on a JSON extracted from the data archive:
python export_constraints.py --input <instance.json> --output <model.lp>
"""
import argparse,hashlib,json,math
from pathlib import Path
def export(source,target):
 d=json.loads(source.read_text(encoding='utf-8-sig'));jobs=d['jobs'];resources=d['resources']
 ids={r['id']:i for i,r in enumerate(resources)};assert len(ids)==len(resources)
 assert len({j['id'] for j in jobs})==len(jobs)
 columns=[[] for _ in resources]
 for i,j in enumerate(jobs):
  assert math.isfinite(j['value']) and j['value']>0
  assert j['resources'] and len(j['resources'])==len(set(j['resources']))
  for v in j['resources']:columns[ids[v]].append(i)
 for v in resources:assert isinstance(v['capacity'],int) and v['capacity']>=0
 target.parent.mkdir(parents=True,exist_ok=True)
 with target.open('x',encoding='utf-8') as f:
  f.write('\\ Full static packing instance; mapping retains original IDs.\nMaximize\n objective:\n')
  for i,j in enumerate(jobs):f.write(f"  + {j['value']} x{i+1}\n")
  f.write('Subject To\n')
  for k,(rr,nn) in enumerate(zip(resources,columns)):
   f.write(f' r{k+1}:')
   if not nn:f.write(' 0 x1')
   for offset in range(0,len(nn),12):f.write('\n  '+' + '.join(f'x{i+1}' for i in nn[offset:offset+12])+(' +' if offset+12<len(nn) else ''))
   f.write(f" <= {rr['capacity']}\n")
  f.write('Binary\n')
  for i in range(len(jobs)):f.write(f' x{i+1}\n')
  f.write('End\n')
 mapping=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),rows={f'r{i+1}':v['id'] for i,v in enumerate(resources)},variables={f'x{i+1}':j['id'] for i,j in enumerate(jobs)},constraints=len(resources),binary_variables=len(jobs),incidences=sum(map(len,columns)),scope='Every original resource row retained, including redundant and empty rows; no solver preprocessing.')
 target.with_suffix(target.suffix+'.mapping.json').write_text(json.dumps(mapping,ensure_ascii=False,indent=2),encoding='utf-8')
 return mapping
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();print(json.dumps(export(a.input,a.output),ensure_ascii=False))
