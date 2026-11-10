"""Continuous simultaneous-flight separation using Bernstein subdivision."""
from run_engineering_full_joint import *
from Agent.Publication.engineering_free_trajectory import split_controls
import math

def crop(c,left,right):
    if right<1:c=split_controls(c,right)[0]
    if left>0:c=split_controls(c,left/right)[1]
    return c

def separate(a,b,required,left,right,depth=0):
    d=a-b;low=d.min(axis=0);high=d.max(axis=0);nearest=np.maximum(low,np.minimum(0,high));bound=float(np.linalg.norm(nearest))
    if bound>=required-1e-8:return True,bound,None,depth
    mid=eval_septic(d,right-left,.5)[0];distance=float(np.linalg.norm(mid))
    if distance<required-1e-7:return False,bound,dict(time_s=(left+right)/2,distance_m=distance,interval_s=[left,right]),depth
    if depth>=22 or right-left<1e-6:return False,bound,dict(interval_s=[left,right],unresolved=True),depth
    al,ar=split_controls(a);bl,br=split_controls(b);middle=(left+right)/2
    p1,b1,f1,d1=separate(al,bl,required,left,middle,depth+1)
    if not p1:return p1,b1,f1,d1
    p2,b2,f2,d2=separate(ar,br,required,middle,right,depth+1);return p2,min(b1,b2),f2,max(d1,d2)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--iteration',type=int,default=0);parser.add_argument('--tag');args=parser.parse_args();tag=args.tag or f'full_joint_retimed_schedule_{args.iteration}'
    solution=read(OUT/(tag+'.json'));scene,pilot,terrain,weather=context();required=2*scene['joint_reference_profile']['aircraft']['body_radius_m']+scene['joint_reference_profile']['aircraft']['inter_uav_clearance_m']
    sorties=[read(OUT/(tag+f'_sortie{s["slot"]}.json')) for s in solution['sorties']];fail=[];checked=0;minbound=math.inf;maxdepth=0;cuts=read(OUT/'full_joint_airspace_cuts.json') if args.iteration else []
    for ia,A in enumerate(sorties):
        for B in sorties[ia+1:]:
            for a in A['timeline']:
                for b in B['timeline']:
                    a0=A['source_sortie']['launch_s']+a['start_s'];a1=A['source_sortie']['launch_s']+a['end_s'];b0=B['source_sortie']['launch_s']+b['start_s'];b1=B['source_sortie']['launch_s']+b['end_s']
                    left,right=max(a0,b0),min(a1,b1)
                    if right<=left:continue
                    ac=crop(np.array(a['control_points_m']),(left-a0)/(a1-a0),(right-a0)/(a1-a0));bc=crop(np.array(b['control_points_m']),(left-b0)/(b1-b0),(right-b0)/(b1-b0))
                    ok,bound,violation,depth=separate(ac,bc,required,left,right);checked+=1;minbound=min(minbound,bound);maxdepth=max(maxdepth,depth)
                    if not ok:
                        fail.append(dict(sorties=[A['source_sortie']['slot'],B['source_sortie']['slot']],arc_a=a['source_arc'],arc_b=b['source_arc'],**violation))
                        # Restrict only this conflicting physical segment pair
                        # to non-overlap; an explicitly conservative branch,
                        # not a valid exclusion of all original feasible pairs.
                        def local_interval(seg,flight):
                            arc=seg['source_arc'];i=arc[1]
                            base=0 if i==9 else flight['source_sortie']['capture_times_s'][str(i)]-flight['source_sortie']['launch_s']+next(c['end_s']-c['start_s'] for c in flight['captures'] if c['task_index']==i)
                            return [seg['start_s']-base,seg['end_s']-base]
                        c=dict(arc_a=a['source_arc'],arc_b=b['source_arc'],occupancy_a_s=local_interval(a,A),occupancy_b_s=local_interval(b,B),role='conservative temporal branch for two verified colliding curve pieces')
                        if c not in cuts:cuts.append(c)
    save(OUT/(tag+'_airspace.json'),dict(observed_at=stamp(),schedule_sha256=sha(OUT/(tag+'.json')),required_center_separation_m=required,continuous_pairwise_pass=not fail,
        checked_overlapping_segment_pairs=checked,min_certified_box_bound_m=minbound if math.isfinite(minbound) else None,maximum_subdivision_depth=maxdepth,failures=fail))
    if fail:save(OUT/'full_joint_airspace_cuts.json',cuts)
    print(json.dumps(dict(pass_all=not fail,checked=checked,failures=len(fail),cuts=len(cuts),min_bound_m=minbound)))
    if fail:raise SystemExit(1)

if __name__=='__main__':main()
