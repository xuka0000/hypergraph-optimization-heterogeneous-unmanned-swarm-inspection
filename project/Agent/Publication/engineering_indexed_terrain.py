"""Full source DEM with logarithmic selectors and exact deferred affine rows.

No local crop, nearest-triangle substitution, smoothing or hull extrapolation.
At an edge, either adjacent affine cell is admissible; derivative acceptance
still requires both one-sided states in the continuous trajectory verifier.
"""
import numpy as np
from Agent.Publication.engineering_indexed_constraints import AffineRow


class IndexedTerrain:
    def __init__(self, terrain, origin_xy):
        self.terrain = terrain
        self.origin = np.asarray(origin_xy, dtype=float)
        self.xy = np.asarray(terrain.tri.points)-self.origin
        self.z = np.asarray(terrain.values).reshape(-1)
        self.cells = []
        for k, ids in enumerate(terrain.tri.simplices):
            T = terrain.tri.transform[k]
            shifted_base = T[2]-self.origin
            gradient = (self.z[ids[:2]]-self.z[ids[2]]) @ T[:2]
            intercept = self.z[ids[2]]-gradient @ shifted_base
            bary_constant = -T[:2] @ shifted_base
            self.cells.append(dict(vertices=ids, gradient=gradient, intercept=float(intercept),
                                   bary_matrix=T[:2], bary_constant=bary_constant))
        self.slope_min = np.min([c['gradient'] for c in self.cells], axis=0)
        self.slope_max = np.max([c['gradient'] for c in self.cells], axis=0)
        self.seed_entries=[]

    def add(self, model, relations, xy, name):
        cell = relations.selector(name+'_cell', len(self.cells))
        ground = model.addVar(name+'_z', lb=float(self.z.min()), ub=float(self.z.max()))
        weights = [model.addVar(name+f'_weight{k}', lb=0., ub=1.) for k in range(3)]
        model.addCons(sum(weights) == 1., name=name+'_barycentric_sum')
        slope = [model.addVar(name+f'_slope{q}', lb=float(self.slope_min[q]),
                              ub=float(self.slope_max[q])) for q in range(2)]

        def rows(k):
            c = self.cells[k]
            ids, g = c['vertices'], c['gradient']
            # Local simplex weights avoid multiplying near-collinear-cell
            # inverse matrices by large UTM offsets. Three weights are shared
            # across all candidate cells, not allocated for each cell.
            ans = [AffineRow('xy_'+str(q), [(1.,xy[q])]+
                             [(-float(self.xy[v,q]),weights[j]) for j,v in enumerate(ids)],0.,0.)
                   for q in range(2)]
            ans.append(AffineRow('height', [(1.,ground)]+
                        [(-float(self.z[v]),weights[j]) for j,v in enumerate(ids)],0.,0.))
            ans += [AffineRow('slope_'+str(q), [(1.,slope[q])],float(g[q]),float(g[q])) for q in range(2)]
            return ans

        relations.add(cell, rows, [*xy,ground,*slope,*weights], name=name)
        result=dict(cell=cell, ground=ground, slopes=slope, weights=weights, row_factory=rows)
        self.seed_entries.append((xy,result))
        return result

    def reseed(self,model,solution):
        """Map a proposed seed to this triangulation, without fixing decisions."""
        maximum_height_change=0.
        for xy,data in self.seed_entries:
            local=np.array([model.getSolVal(solution,v) for v in xy])
            k=int(self.terrain.tri.find_simplex(local+self.origin))
            if k<0:raise ValueError('Mapped terrain seed outside complete source hull')
            selector=data['cell'];model.setSolVal(solution,selector.value,k)
            for bit,v in enumerate(selector.bits):model.setSolVal(solution,v,(k>>bit)&1)
            cell=self.cells[k];ids=cell['vertices']
            weights=np.linalg.solve(np.vstack([self.xy[ids].T,np.ones(3)]),np.r_[local,1.])
            for v,value in zip(data['weights'],weights):model.setSolVal(solution,v,float(value))
            ground=float(weights@self.z[ids]);maximum_height_change=max(maximum_height_change,abs(ground-model.getSolVal(solution,data['ground'])))
            model.setSolVal(solution,data['ground'],ground)
            for v,value in zip(data['slopes'],cell['gradient']):model.setSolVal(solution,v,float(value))
        return dict(entries=len(self.seed_entries),maximum_ground_change_m=maximum_height_change,
                    original_decisions_fixed=0,accepted=False,requires_complete_candidate_revalidation=True)

    def evaluate_cell(self, category, xy):
        c = self.cells[category]
        bary = c['bary_matrix'] @ np.asarray(xy)+c['bary_constant']
        return dict(height=float(c['gradient'] @ xy+c['intercept']),
                    gradient=c['gradient'].tolist(),
                    barycentric=[*bary.tolist(),float(1-bary.sum())])
