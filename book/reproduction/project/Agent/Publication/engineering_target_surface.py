"""Explicit derived reference-surface repair; immutable OSM source untouched.

The 1cm grid is a template construction choice, not measured tower precision.
All source wire tubes are checked. No target/host obstacle is excluded.
"""
import numpy as np
from Agent.Publication.engineering_regional_safety import wire_closest


class WireSurfaceAdapter:
    def __init__(self,wires,margin=.01,step=.01):
        self.wires=wires;self.margin=margin;self.step=step
        endpoints=np.asarray([w['endpoint_xyz_m'] for w in wires],float)
        radii=np.array([w['radius_m'] for w in wires]);sag=np.array([w['sag_m'] for w in wires])
        self.lo=endpoints.min(axis=1)-radii[:,None];self.lo[:,2]-=sag
        self.hi=endpoints.max(axis=1)+radii[:,None]

    def violations(self,points,margin=None):
        margin=self.margin if margin is None else margin;fail=[]
        for j,p in enumerate(points):
            ids=np.flatnonzero(np.all(p>=self.lo-margin-1e-8,axis=1)&np.all(p<=self.hi+margin+1e-8,axis=1))
            for k in ids:
                distance,parameter=wire_closest(p,self.wires[k])
                if distance<margin-1e-8:fail.append(dict(sample=j,wire=int(k),wire_id=self.wires[k]['id'],distance_m=distance,parameter=parameter))
        return fail

    def repair(self,points,normal,max_reference_shift_m=5.):
        points=np.asarray(points,float);normal=np.asarray(normal,float)
        initial=self.violations(points)
        if not initial:return 0.,dict(initial_conflicts=[],grid_steps=0,final_clearance_margin_m=self.margin)
        for step in range(1,int(max_reference_shift_m/self.step)+1):
            delta=step*self.step
            if not self.violations(points+delta*normal):
                return delta,dict(initial_conflicts=initial,grid_steps=step,grid_m=self.step,final_clearance_margin_m=self.margin,
                    reason='template patch intersects source wire tube or its1cm construction margin')
        # This is a construction failure, never a modified feasible domain or
        # permission to drop a source obstacle / affected observation.
        raise ValueError('No source-safe target surface within diagnostic5m repair bound')
