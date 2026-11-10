"""Post-hoc diagnosis of saved trajectories; no simulations or fitting."""
import numpy as np
import pandas as pd
from run_weather_mission_coupling import OUT,SRC,read,save,ref,now

def main():
    mission=read(OUT/'mission.json');results=read(OUT/'results.json');rows=[]
    for case in read(OUT/'cases.json'):
        s=pd.read_csv(OUT/'predictions'/(case['id']+'_states.csv'))
        s=s[s.time>=0];phases=[]
        for ph in mission['phases']:
            a=s[(s.time>=ph['begin_s']-1e-6)&(s.time<=ph['end_s']+1e-6)]
            assert len(a)>=2
            phases.append({**ph,'samples_including_both_endpoints':len(a),
              'tracking_rms_m':float(np.sqrt(np.mean(a.tracking_error_m**2))),
              'tracking_max_m':float(a.tracking_error_m.max()),
              'tracking_end_m':float(a.tracking_error_m.iloc[-1])})
        photos=[p for p in phases if p['phase']=='photo'];assert len(photos)==9
        energy=[r for r in results['rows'] if r['case_id']==case['id']]
        rows.append({'case_id':case['id'],'phases':phases,
          'transfer_max_m':max(p['tracking_max_m'] for p in phases if p['phase'] in ['outbound','return']),
          'photo_max_m':max(p['tracking_max_m'] for p in photos),
          'photo_end_max_m':max(p['tracking_end_m'] for p in photos),
          'energy_min_Wh':min(r['energy_Wh'] for r in energy),'energy_max_Wh':max(r['energy_Wh'] for r in energy),
          'delta_pct_by_model':{r['model']:r['delta_vs_calm15_pct'] for r in energy},
          'photo_acceptance_pass':None})
    save(OUT/'trajectory_diagnosis.json',{'observed_at':now(),'kind':'post_hoc_saved_output_diagnosis','cases':rows,
      'inputs':[ref(OUT/'mission.json'),ref(OUT/'results.json')],
      'source_audit':{'author_source':ref(SRC/'QuadSim.py'),
        'controller_lines':'239-257: cascaded P velocity / D acceleration, no feedforward; horizontal desired-velocity cap5m/s',
        'author_propagate_lines':'301-327: lookahead ghost target, path-index advances only within speed*ghost_delta; not used by current fixed-time diagnostic',
        'current_adapter':'Every target advances on frozen0.1s timeline to preserve common prescribed mission duration. This differs from author propagate; original dynamics/controller only are reused.',
        'inference':'Finite acceleration, drag and absent velocity feedforward at a5m/s reference and5m/s command cap contribute to transfer lag. The fixed-time caller advances even when lagging. This is a simulator/task adapter limitation, not measured aircraft performance.',
        'energy_model_paper':{'title':'CVaR-based Flight Energy Risk Assessment for Multirotor UAVs using a Deep Energy Model','year':2021,'conference':'ICRA','doi':'10.1109/ICRA48506.2021.9561658','verified_primary_url':'https://ieeexplore.ieee.org/abstract/document/9561658'}},
      'decision':'Do not use these fixed-duration totals as energy to complete a validated inspection. All60 conditional predictions retained; no claim that wind saves energy or temperature has negligible battery effect.',
      'next_dependency':'A completion-aware task adapter with source-bound camera/position acceptance and actual elapsed flight time, before using energy in the scheduler; measured wind/attitude/power data still needed for accuracy.',
      'new_experiments':0,'user_acceptance_threshold_invented':False})
    print('Diagnosed all12 saved trajectories and108 photo phases; no new experiment')

if __name__=='__main__':main()
