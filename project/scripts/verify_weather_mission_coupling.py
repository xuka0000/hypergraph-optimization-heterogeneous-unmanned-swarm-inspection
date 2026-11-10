"""Independent saved-state, feature, energy and scope verification."""
import json,math,pathlib
import numpy as np,pandas as pd
from run_weather_mission_coupling import ROOT,OUT,BASE,SRC,WEATHER,TASK,MODELS,read,save,ref,now

def main():
    result=read(OUT/'results.json');protocol=read(OUT/'protocol.json');mission=read(OUT/'mission.json');cases=read(OUT/'cases.json');audit=read(BASE/'cmu_pretrained/input_audit.json')
    assert result['status']=='complete_conditional_mission_diagnostic' and len(result['rows'])==60
    assert len({(x['case_id'],x['model']) for x in result['rows']})==60
    for asset in protocol['protected_author_assets']:assert ref(ROOT/asset['path'])==asset
    for name in ['mission','cases','entrypoint','adapter']:
        r=read(OUT/'execution_binding.json')[name];assert ref(ROOT/r['path'])==r
    source=read(ROOT/mission['source_scene']['path']);route=next(x for x in source['tower_routes'] if x['tower_id']=='T001')
    photos=[x for x in mission['phases'] if x['phase']=='photo'];assert len(photos)==9
    ids={x['id'] for x in source['inspection_tasks'] if x['tower_id']=='T001'};assert {p['source_id'] for p in photos}==ids
    assert abs(sum(x['unrounded_duration_s'] for x in mission['phases'] if x['phase'] in ['photo','local_transfer'])-300)<1e-9
    times=np.array([p['time_s'] for p in mission['points']]);assert np.all(np.diff(times)>0) and times[-1]==mission['duration_s']
    phase_duration=sum(p['end_s']-p['begin_s'] for p in mission['phases']);assert abs(phase_duration-times[-1])<1e-10
    lo=np.array(audit['data_min'])[:5];hi=np.array(audit['data_max'])[:5];dr=read(WEATHER/'model.json')['CMU_trainval_density_range_kgm3']
    maxima={'position_update_m':0.,'velocity_force_update_ms':0.,'air_triangle_ms':0.,'energy_csv_vs_saved_Wh':0.}
    verified=[]
    for c in cases:
        s=pd.read_csv(OUT/'predictions'/(c['id']+'_states.csv'));p=pd.read_csv(OUT/'predictions'/(c['id']+'_power.csv'))
        pos=s[['pos_n','pos_w','pos_u']].values;vel=s[['vel_n','vel_w','vel_u']].values;air=s[['air_n','air_w','air_u']].values
        target=s[['target_n','target_w','target_u']].values;dt=np.diff(s.time.values)
        maxima['position_update_m']=max(maxima['position_update_m'],float(abs(np.diff(pos,axis=0)-vel[:-1]*dt[:,None]).max()))
        maxima['air_triangle_ms']=max(maxima['air_triangle_ms'],float(abs(air-(vel-np.array(c['wind_nwu_ms']))).max()))
        # Independently reconstruct original controller thrust and next-step force.
        err=target[:-1]-pos[:-1];n=np.linalg.norm(err[:,:2],axis=1);scale=np.divide(np.minimum(n,5),n,out=np.zeros_like(n),where=n!=0)
        err[:,:2]*=scale[:,None];acc_cmd=3*(err-vel[:-1]);acc_cmd[:,2]+=9.81;thrust=acc_cmd[:,2]*3.71
        phi=s.roll.values[1:];theta=s.pitch.values[1:]
        acc=np.column_stack([np.sin(theta)*np.cos(phi)*thrust/3.71,-np.sin(phi)*thrust/3.71,np.cos(theta)*np.cos(phi)*thrust/3.71-9.81])
        acc-=.5*.65*c['density_kgm3']*.25*np.square(air[:-1])*np.sign(air[:-1])/3.71
        maxima['velocity_force_update_ms']=max(maxima['velocity_force_update_ms'],float(abs(np.diff(vel[1:],axis=0)-acc[:-1]*dt[1:,None]).max()))
        body_x=air[:,0]*np.cos(s.yaw.values)+air[:,1]*np.sin(s.yaw.values);body_y=-air[:,0]*np.sin(s.yaw.values)+air[:,1]*np.cos(s.yaw.values)
        assert np.allclose(body_x,s.airspeed_x,rtol=1e-12,atol=1e-12) and np.allclose(body_y,s.airspeed_y,rtol=1e-12,atol=1e-12)
        assert np.allclose(s.aoa,s.roll,atol=1e-12) and np.allclose(np.hypot(body_x,body_y),s.airspeed,atol=1e-12)
        retained=s[s.time>=0];assert np.array_equal(p.time_s.values,retained.time.values)
        feats=retained[['airspeed_x','airspeed_y','vertspd','aoa','airspeed']].values;ood=(feats<lo)|(feats>hi)
        diag=next(x for x in result['case_diagnostics'] if x['case_id']==c['id']);assert diag['varying_ood_rows']==int(ood.any(axis=1).sum())
        assert diag['density_ood']==(not dr[0]<=c['density_kgm3']<=dr[1])
        for model in MODELS:
            r=next(x for x in result['rows'] if x['case_id']==c['id'] and x['model']==model)
            # Roundtrip original float32 network output; physical model float64.
            power=p[model+'_W'].values.astype('float32' if model!='Liu' else 'float64').astype('float64')
            area=(power[:-1]+power[1:])*.5*np.diff(p.time_s.values)/3600
            e=float(area.sum());diff=abs(e-r['energy_Wh']);maxima['energy_csv_vs_saved_Wh']=max(maxima['energy_csv_vs_saved_Wh'],diff)
            assert diff<1e-5
            midpoint=(p.time_s.values[:-1]+p.time_s.values[1:])*.5;phases={}
            for ph in mission['phases']:
                selection=(midpoint>=ph['begin_s'])&(midpoint<ph['end_s']);phases[ph['phase']]=phases.get(ph['phase'],0.)+float(area[selection].sum())
            for k,v in phases.items():assert abs(v-r['phase_energy_Wh'][k])<1e-7
            assert r['reference_budget_pass']==(e<=79.92) and abs(r['reference_margin_Wh']-(79.92-e))<1e-5
            assert r['weather_base_pass']==c['weather_screen']['base_screen_pass']
            assert r['joint_reference_screen']==(e<=79.92 and c['weather_screen']['base_screen_pass'])
            assert r['operational_flight_feasible'] is None and r['ground_truth_Wh'] is None and r['accuracy_error_pct'] is None
            e0=next(x['energy_Wh'] for x in result['rows'] if x['case_id']=='calm_15C' and x['model']==model)
            assert abs(r['delta_vs_calm15_Wh']-(r['energy_Wh']-e0))<1e-10
            verified.append({'case_id':c['id'],'model':model,'independent_energy_Wh':e,'difference_Wh':diff})
    assert maxima['position_update_m']<1e-9 and maxima['velocity_force_update_ms']<1e-9 and maxima['air_triangle_ms']<1e-9
    save(OUT/'delivery_validation.json',{'observed_at':now(),'passed_for_current_scope':True,'pairs':60,'cases':12,'models':5,'source_assets_unchanged':True,'common_interval_s':mission['duration_s'],
      'photo_tasks':9,'all_input_and_phase_identities_verified':True,'independent_state_and_energy_max_errors':maxima,'verified_results':verified,
      'science_limits':['Real map coordinates plus reference task geometry and simulated states; no measured power ground truth',
        'Actual weather cases use uniform60m regional background; no tower-scale CFD/height profile/density correction',
        'Reference79.92Wh is nominal99.9Wh with inherited20percent reserve; excludes takeoff/landing and temperature derating',
        'Field feasibility and error remain unknown; feature-envelope checks are necessary but not sufficient for distribution validity',
        'The corrected frame adapter matches measured source definitions; high-accuracy transfer still requires flight validation'],
      'verifier':ref(pathlib.Path(__file__))})
    print(json.dumps({'passed':True,'pairs':len(verified),'maximum_errors':maxima},ensure_ascii=False))

if __name__=='__main__':main()
