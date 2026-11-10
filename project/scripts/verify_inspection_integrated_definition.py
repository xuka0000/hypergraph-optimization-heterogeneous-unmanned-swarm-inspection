"""Check model bindings and analytic identities; no flight/optimization experiment."""
from pathlib import Path
import ast, datetime, hashlib, json, math, re, subprocess, sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from Agent.Publication.engineering_model_integration import readiness,compatible_power_model


def main():
    rows=[]
    def check(name,ok,detail):
        rows.append(dict(id=name,passed=bool(ok),detail=detail))
    def read(s):return json.loads((ROOT/s).read_text('utf-8-sig'))
    def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
    contract=read('Agent/Publication/engineering_inspection_contract.json')
    tex=(ROOT/contract['mathematical_source']).read_text('utf-8')
    labels=re.findall(r'\\label\{([^}]+)\}',tex)
    required={s for row in contract['constraint_families'] for s in row['equation_labels']}
    check('all_constraint_labels_bound',required<=set(labels),dict(missing=sorted(required-set(labels)),families=len(contract['constraint_families']),labels=len(labels)))
    check('no_duplicate_equation_labels',len(labels)==len(set(labels)),len(labels))
    variable_ids=[v['id'] for v in contract['variables']]
    invalid_dependencies={f['id']:sorted(set(f.get('depends_on',[]))-set(variable_ids))
        for f in contract['constraint_families'] if set(f.get('depends_on',[]))-set(variable_ids)}
    missing_dependencies=[f['id'] for f in contract['constraint_families'] if not f.get('depends_on')]
    check('variable_block_dependencies_resolve',not invalid_dependencies and not missing_dependencies,
          dict(undefined=invalid_dependencies,missing=missing_dependencies,scope='block dependency registration, not scalar hypergraph completeness'))
    check('unique_variable_block_ids',len(variable_ids)==len(set(variable_ids)),len(variable_ids))
    check('mathematical_source_identity',sha(ROOT/contract['mathematical_source'])==contract['mathematical_source_sha256'],contract['revision'])
    for row in contract['integration_revision']['sources']:
        check('source_'+row['id'],sha(ROOT/row['path'])==row['sha256'],row['path'])
    protocol=read('总结/03_实验数据/天气因素建模/航段能耗联算/protocol.json')
    protected=protocol['protected_author_assets']
    identities=[sha(ROOT/r['path'])==r['sha256'] for r in protected]
    check('original_author_code_and_weights_unchanged',all(identities),dict(assets=len(identities),mismatches=[r['path'] for r,ok in zip(protected,identities) if not ok]))
    scene_rows=[]
    for row in contract['source_scenes']:
        ok=sha(ROOT/row['path'])==row['sha256'];scene_rows.append(dict(scene=row['scene_id'],unchanged=ok))
    check('source_scenes_preserved',all(r['unchanged'] for r in scene_rows),scene_rows)
    scene=read('总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json')
    check('full_geographic_scope_preserved',len(scene['inspection_tasks'])==2133,dict(tasks=len(scene['inspection_tasks']),tower_count=237,stops=len(scene['stops'])))
    M=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
    check('ENU_NWU_is_proper_rotation',np.allclose(M@M.T,np.eye(3)) and np.isclose(np.linalg.det(M),1),M.tolist())
    errors=[]
    for psi in [0.,.7,math.pi/2,-2.]:
        air=np.array([3.,-2.,1.]);body_direct=np.array([[math.cos(psi),math.sin(psi)],[-math.sin(psi),math.cos(psi)]])@air[:2]
        psiN=psi-math.pi/2;body_nwu=np.array([[math.cos(psiN),math.sin(psiN)],[-math.sin(psiN),math.cos(psiN)]])@(M@air)[:2]
        errors.append(float(np.max(np.abs(body_direct-body_nwu))))
    check('heading_features_frame_invariant',max(errors)<1e-12,dict(max_error=max(errors)))
    # Battery analytic example: stable high-voltage branch, sign and unit checks.
    voc,vpol,r0,r1,c1=24.,.2,.05,.1,100.
    p=480.;A=voc-vpol;disc=A*A-4*r0*p
    I=(A-math.sqrt(disc))/(2*r0);U=A-r0*I
    check('battery_terminal_power_identity',abs(U*I-p)<1e-10,dict(current_A=I,voltage_V=U,power_W=U*I))
    qAh=10.;dt=60.;dz=-I*dt/(3600*qAh)
    check('SOC_coulomb_units',dz<0 and abs(dz-(-I/600))<1e-12,dict(delta_SOC=dz))
    # Rest: residual polarization dissipates and must still produce heat.
    check('polarization_rest_cooling_not_free_reset',vpol*vpol/r1>0 and -vpol/(r1*c1)<0,dict(heat_W=vpol*vpol/r1,dpolarization_V_per_s=-vpol/(r1*c1)))
    grid,eta,load=1000.,.9,0.;inWh=eta*grid*dt/3600;outWh=0.
    check('charging_efficiency_once',abs(eta*grid*dt-(load*dt+3600*(inWh-outWh)))<1e-9,dict(grid_Wh=grid*dt/3600,terminal_in_Wh=inWh))
    m,g=3.71,9.81;f=np.ones(4)*m*g/4
    check('rigid_body_hover_equilibrium',abs(f.sum()-m*g)<1e-12,dict(total_thrust_N=float(f.sum()),rotor_count=4))
    # H must join the observed prefix with planned future, never zero-reset.
    history=np.arange(100.).reshape(20,5);new=np.array([2.,3.,4.,5.,6.]);shift=np.vstack([history[1:],new])
    check('20_sample_causal_boundary',np.array_equal(shift[:-1],history[1:]) and np.array_equal(shift[-1],new),dict(history_shape=list(shift.shape),future_samples_read=0))
    check('no_cross_platform_calibration',compatible_power_model('DJI_Matrice_100','DJI_Matrice_100') and not compatible_power_model('DJI_Matrice_300','DJI_Matrice_100'),'M100 cannot become calibrated M300')
    record=ROOT/'总结/03_实验数据/工程化联合求解/regional_joint/run_manifest.json';before=sha(record)
    cp=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/run_inspection_regional.py'),'--solve'],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=20)
    check('actual_old_executor_rejects_complete_label',cp.returncode==2 and 'integrated_definition_not_yet_executable' in cp.stdout,dict(exit_code=cp.returncode,stdout=cp.stdout.strip(),stderr=cp.stderr[-500:]))
    check('guard_did_not_replace_actual_run_record',sha(record)==before,before)
    check('explicit_old_diagnostic_identity',readiness(ROOT,True)['scope']=='legacy_deterministic_day_ahead_diagnostic_only','checked identity only; did not launch old diagnostic')
    for p in ['scripts/refine_inspection_integrated_model.py','scripts/run_inspection_regional.py','Agent/Publication/engineering_model_integration.py','scripts/build_engineering_inspection_contract.py']:
        ast.parse((ROOT/p).read_text('utf-8-sig'))
    check('edited_python_syntax',True,'AST parsing; no bytecode or solver build')
    result=dict(observed_at=datetime.datetime.now().astimezone().isoformat(),task='inspection_integrated_model_refinement',
        scope='source identity, formula mapping and analytic interface checks only; no runtime integration/full physics validation',
        checks=rows,passed=sum(r['passed'] for r in rows),total=len(rows),all_passed=all(r['passed'] for r in rows),
        definition_sha256=contract['mathematical_source_sha256'],new_optimization_runs=0,
        complete_numeric_instance_bound=False,full_joint_solver_verified=False,closed_loop_verified=False)
    (ROOT/'research_quality/inspection_integrated_definition_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ['passed','total','all_passed','new_optimization_runs']},ensure_ascii=False))
    if not result['all_passed']:
        print(json.dumps([r for r in rows if not r['passed']],ensure_ascii=False));return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
