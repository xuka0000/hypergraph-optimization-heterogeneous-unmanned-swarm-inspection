"""Export a reviewable learning delivery, preserving the authoritative workspace."""
from pathlib import Path
import ast, datetime, hashlib, json, os, re, shutil, sys, zipfile

ROOT = Path(__file__).resolve().parents[1]
BOOK = ROOT / '总结/超图优化与无人集群巡检建模专著'
DELIVERY = BOOK / 'delivery'
REPO = DELIVERY / 'repository'

def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''): h.update(b)
    return h.hexdigest()

def write(p, x):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(x, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')

def copy(src, dst):
    if src.is_symlink(): raise ValueError(f'Symlink not allowed: {src}')
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)

def files(d):
    return sorted(p for p in d.rglob('*') if p.is_file() and '__pycache__' not in p.parts)

def make_archive(name, paths):
    target = REPO/'data'/name
    target.parent.mkdir(parents=True, exist_ok=True)
    members=[]
    with zipfile.ZipFile(target, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in sorted(set(paths)):
            rel=p.relative_to(ROOT).as_posix()
            z.write(p,rel)
            members.append(dict(path=rel,bytes=p.stat().st_size,sha256=sha(p)))
    return dict(path=target.relative_to(REPO).as_posix(),sha256=sha(target),files=members)

def main():
    if REPO.exists(): raise SystemExit('Delivery tree already exists; inspect it instead of overwriting.')
    REPO.mkdir(parents=True)
    # Retain the frozen reproduction package byte-for-byte, including upstream licenses.
    for directory in ['reproduction','sections','figures']:
        for p in files(BOOK/directory): copy(p,REPO/'book'/p.relative_to(BOOK))
    for name in ['main.tex','preamble.tex','references.tex']:
        copy(BOOK/name,REPO/'book'/name)
    for p in (BOOK/'generated').glob('*.tex'):
        if not p.name.startswith(('section_','code_')): copy(p,REPO/'book/generated'/p.name)
    for p in files(BOOK/'evidence'):
        if p.name == 'cross_bridge_heterogeneous_discussion.json': continue
        if p.name.endswith('_excerpt.txt') or p.name=='bmatching_table3.txt': continue
        copy(p,REPO/'book'/p.relative_to(BOOK))
    copy(BOOK/'output/超图优化与无人集群巡检建模.pdf',REPO/'book/monograph.pdf')
    # Current code is a separately identified portable snapshot, never relabelled as an old run.
    frozen=BOOK/'reproduction/project'
    for p in files(frozen): copy(p,REPO/'project'/p.relative_to(frozen))
    selected=set()
    index=json.loads((BOOK/'reproduction/current_source_index.json').read_text(encoding='utf-8'))
    selected.update(ROOT/x['path'] for x in index['sources'])
    for p in files(frozen):
        live=ROOT/p.relative_to(frozen)
        if live.is_file() and p.suffix in {'.py','.tex','.json'} and not str(p.relative_to(frozen)).startswith('总结'):
            selected.add(live)
    selected.update((ROOT/'Agent/Publication').glob('engineering_*'))
    selected.update((ROOT/'Agent/Publication').glob('*weather*.py'))
    selected.update((ROOT/'Agent/Publication').glob('*battery*.py'))
    selected.update((ROOT/'Agent/Publication').glob('*energy*.py'))
    selected.update((ROOT/'Agent/Publication').glob('inspection_reference_*.json'))
    selected.update((ROOT/'scripts').glob('validate_inspection_*.py'))
    selected.update((ROOT/'scripts').glob('*battery*.py'))
    selected.update((ROOT/'scripts').glob('*weather*.py'))
    selected.add(ROOT/'scripts/verify_heterogeneous_teaching_examples.py')
    # weather_factors loads two upstream functions via AST instead of a Python import.
    selected.add(ROOT/'总结/03_实验数据/电池能耗公开机型验证/sources/cmu2021/MetarHandler.py')
    selected={p for p in selected if p.is_file() and p.suffix in {'.py','.json','.tex'}}
    # Follow local static imports; native public author trees are already in the frozen package.
    examined=set(); external=set()
    while selected-examined:
        p=next(iter(selected-examined)); examined.add(p)
        if p.suffix!='.py': continue
        tree=ast.parse(p.read_text(encoding='utf-8-sig'))
        for node in ast.walk(tree):
            modules=[]
            if isinstance(node,ast.Import): modules=[a.name for a in node.names]
            elif isinstance(node,ast.ImportFrom):
                mod=node.module or ''
                modules=[mod]+[mod+'.'+a.name for a in node.names if a.name!='*']
            for module in modules:
                rel=Path(*module.split('.'))
                candidates=[ROOT/rel.with_suffix('.py'), ROOT/rel/'__init__.py', p.parent/rel.with_suffix('.py'),ROOT/'scripts'/rel.with_suffix('.py')]
                matches=[q for q in candidates if q.is_file() and q.is_relative_to(ROOT)]
                if matches: selected.update(matches)
                elif module: external.add(module.split('.')[0])
    current=[]
    for p in sorted(selected):
        rel=p.relative_to(ROOT)
        copy(p,REPO/'project'/rel)
        current.append(dict(path=rel.as_posix(),sha256=sha(p),bytes=p.stat().st_size))
    for name in ['LICENSE','README.md']:
        rel=Path('总结/03_实验数据/电池能耗公开机型验证/sources/cmu2021')/name
        copy(ROOT/rel,REPO/'project'/rel)
    write(REPO/'CURRENT_SOURCES.json',dict(observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),files=current,external_import_roots=sorted(external),scientific_changes=0))
    # Small fresh evidence is outside immutable historical archive members.
    archives=[]
    scene=ROOT/'总结/03_实验数据/真实地理巡检场景共享'
    archives.append(make_archive('geographic_scenes.zip',files(scene)))
    validation=ROOT/'总结/03_实验数据/工程化联合求解/module_validation'
    archives.append(make_archive('component_results.zip',files(validation)))
    energy=ROOT/'总结/03_实验数据/电池能耗公开机型验证'
    weather=ROOT/'总结/03_实验数据/天气因素建模'
    records=[]; withheld=[]
    for d in [energy,weather]:
        for p in files(d):
            rel=p.relative_to(d)
            if any(x in rel.parts for x in ['build','module_pdf_review','__pycache__']):continue
            if p.suffix.lower() in {'.pdf','.png','.jpg','.log'}:continue
            if d==energy and rel.parts[:2] in [('sources','rodrigues_data'),('sources','rodrigues2022')]:
                if p.name=='source_tree.json':
                    copy(p,REPO/'project'/p.relative_to(ROOT))
                    continue
                withheld.append(dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),reason='Redistribution license not established; upstream URLs/commits retained in source_manifest.json.'))
                continue
            records.append(p)
    archives.append(make_archive('energy_weather_evidence.zip',records))
    write(REPO/'data/archives.json',dict(archives=archives))
    write(REPO/'data/external_assets.json',dict(not_redistributed=withheld,reference_pdfs='VPP and third-party article/manual PDFs are not redistributed; see bibliography and original source URLs.'))
    # Existing templates are deliberately plain files, not editor/account automation.
    for p in files(ROOT/'scripts/inspection_study_templates'):
        copy(p,REPO/p.relative_to(ROOT/'scripts/inspection_study_templates'))
    copy(ROOT/'scripts/package_inspection_study.py',REPO/'provenance/package_inspection_study.py')
    write(REPO/'STUDY_STATUS.json',dict(
        observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),pdf_pages=221,
        title='超图优化与异构无人集群巡检',english_title='Hypergraph Optimization and Heterogeneous Unmanned Swarm Inspection',
        full_regional_joint_solve_complete=False,full_execution_replanning_loop_complete=False,
        same_aircraft_measurement_calibration_groups_unresolved=9,
        scope='Current teaching monograph, component code, reference inputs and preserved historical positive/negative evidence. No new research comparison is launched by packaging.',
        historical_version_warning='conditional-613b has one unrecovered exact source hash; frozen package README documents it.'
    ))
    manifest=[dict(path=p.relative_to(REPO).as_posix(),bytes=p.stat().st_size,sha256=sha(p)) for p in files(REPO)]
    write(REPO/'MANIFEST.json',dict(schema_version=1,files=manifest,total_bytes=sum(x['bytes'] for x in manifest)))
    print(json.dumps(dict(repository=str(REPO),files=len(manifest)+1,MiB=sum(x['bytes'] for x in manifest)/2**20,current_sources=len(current),data_archives=[dict(path=a['path'],members=len(a['files'])) for a in archives]),ensure_ascii=False))

if __name__=='__main__':main()
