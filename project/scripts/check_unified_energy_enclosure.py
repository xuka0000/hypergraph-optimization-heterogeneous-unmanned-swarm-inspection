"""Analytical known cases and unchanged inspection initializer energy bounds."""
from run_engineering_full_joint import *
from Agent.Publication.engineering_energy_enclosure import EnergyEnclosure
import math


def main():
    scene,_,terrain,weather=context();engine=EnergyEnclosure(scene,terrain);rows=[]
    receipt=ROOT/'research_quality/inspection_unified_energy_enclosure.json'
    if receipt.exists():
        path=ROOT/'research_quality/inspection_unified_energy_enclosure_attempts.json';history=read(path) if path.exists() else [];history.append(read(receipt));save(path,history)
    point=np.array(scene['fleet']['vehicles'][0]['initial_xyz_m']);point[2]+=40
    def analytic(t):
        wind,rho=weather(point,t,float(terrain(*point[:2])));air=scene['joint_reference_profile']['aircraft'];en=scene['joint_reference_profile']['energy']
        return en['hover_power_W']*math.sqrt(1.225/rho)+.5*air['drag_CdA_m2']*rho*np.linalg.norm(wind)**3
    for start,duration in [(0.,60.),(3590.,20.)]:
        seg=dict(start_s=0.,end_s=duration,control_points_m=[point.tolist()]*8)
        bound=engine.integrate([seg],start,.00001)
        expected=(analytic(start)*duration/3600 if start==0 else (analytic(3599)*10+analytic(3601)*10)/3600)
        rows.append(dict(name='stationary_hour_split' if start else 'stationary_analytic',passed=bool(bound['converged'] and bound['lower_Wh']<=expected<=bound['upper_Wh']),expected_Wh=float(expected),**bound))
        print(rows[-1],flush=True);save(receipt,dict(observed_at=stamp(),checks=rows,all_pass=False,state='running',pid=os.getpid(),create_time=psutil.Process().create_time()))
    for k in range(2):
        path=OUT/f'joint_optical_bound_SOC1_sortie{k}.json';d=read(path);start=time.perf_counter()
        def progress(info):
            print('energy refinement',k,info,flush=True)
            save(receipt,dict(observed_at=stamp(),checks=rows,all_pass=False,state='running',pid=os.getpid(),create_time=psutil.Process().create_time(),current_flight=k,progress=info))
        bound=engine.integrate(d['timeline'],120.,.005,progress=progress)
        save(receipt,dict(observed_at=stamp(),checks=rows,all_pass=False,state='candidate_integral_complete',current_flight=k,integral=bound))
        reference=energy_of(d['timeline'],scene['joint_reference_profile'],terrain,weather,120.)
        rows.append(dict(name=f'unchanged_initializer_flight{k}',passed=bool(bound['converged'] and bound['lower_Wh']<=reference<=bound['upper_Wh']),reference_quadrature_Wh=float(reference),source_sha256=sha(path),seconds=time.perf_counter()-start,**bound))
        print(rows[-1],flush=True);save(receipt,dict(observed_at=stamp(),checks=rows,all_pass=False,state='running',pid=os.getpid(),create_time=psutil.Process().create_time()))
    save(receipt,dict(observed_at=stamp(),all_pass=all(r['passed'] for r in rows),checks=rows,state='complete',inspection_optimization_runs=0,scope='candidate enclosure only; global optimizer integral closure remains open',code_sha256=sha(ROOT/'Agent/Publication/engineering_energy_enclosure.py')))
    if not all(r['passed'] for r in rows):raise SystemExit(1)


if __name__=='__main__':main()
