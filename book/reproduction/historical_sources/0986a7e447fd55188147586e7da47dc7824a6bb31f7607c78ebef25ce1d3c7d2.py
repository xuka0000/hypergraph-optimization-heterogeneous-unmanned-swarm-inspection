"""Recover the original DEM sampling grid before projected triangulation.

Only an exact match to the acquisition script's rounded query coordinates is
restored. Elevations, tower samples and original source JSON remain unchanged.
This changes the derived triangulation and must invalidate old geometric
acceptance until rechecked; it is not an equivalent floating-point surface.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from pyproj import Transformer
from scipy.interpolate import LinearNDInterpolator


class RetainedTriangulation:
    """Original nondegenerate connectivity on restored source-grid vertices."""
    def __init__(self,points,simplices):
        from matplotlib.tri import Triangulation
        self.points=np.asarray(points);self.simplices=np.asarray(simplices,dtype=np.int32)
        vertices=self.points[self.simplices];self.transform=np.empty((len(simplices),3,2))
        self.transform[:,:2]=np.linalg.inv((vertices[:,:2]-vertices[:,2,None]).transpose(0,2,1))
        self.transform[:,2]=vertices[:,2]
        self.mesh=Triangulation(self.points[:,0],self.points[:,1],self.simplices)
        self.finder=self.mesh.get_trifinder()

    def find_simplex(self,xy):
        xy=np.asarray(xy);return self.finder(xy[...,0],xy[...,1])


class RetainedLinearTerrain:
    def __init__(self,points,elevations,simplices):
        self.tri=RetainedTriangulation(points,simplices);self.points=self.tri.points
        self.values=np.asarray(elevations)[:,None]

    def __call__(self,*coordinates):
        xy=np.asarray(coordinates[0]) if len(coordinates)==1 else np.stack(np.broadcast_arrays(*coordinates),axis=-1)
        shape=xy.shape[:-1];points=xy.reshape(-1,2);cells=np.asarray(self.tri.find_simplex(points))
        result=np.full(len(points),np.nan);inside=cells>=0;ids=cells[inside]
        weights=np.einsum('nij,nj->ni',self.tri.transform[ids,:2],points[inside]-self.tri.transform[ids,2])
        weights=np.column_stack([weights,1-weights.sum(axis=1)])
        result[inside]=(weights*self.values[self.tri.simplices[ids],0]).sum(axis=1)
        return result.reshape(shape)


def regional_terrain(scene, root, restore_grid=False):
    tr=Transformer.from_crs(4326,scene['coordinate_reference']['projected'],always_xy=True)
    samples=scene['terrain']['samples']
    x,y=tr.transform([z['location']['lng'] for z in samples],[z['location']['lat'] for z in samples])
    xy=np.column_stack([x,y]);original=xy.copy();z=np.array([p['elevation'] for p in samples])
    receipt=dict(restore_grid=restore_grid,samples=len(samples),elevations_changed=0,
                 source_samples_removed=0,old_surface_equivalence_claim=not restore_grid)
    if restore_grid:
        # Reconstruct exactly the UTM lattice used in prepare_regional_terrain.py,
        # including its domain. Do not infer a smoothing or distance threshold.
        source=Path(root)/'总结/03_实验数据/真实地理巡检场景共享/sources/regional_dem'/scene['scene_id']/'terrain.json'
        raw=source.read_bytes();meta=json.loads(raw)
        if meta['results']!=samples:raise ValueError('DEM source sample/order mismatch')
        spacing=meta['grid_spacing_m'];lo,hi=np.asarray(meta['bounds_utm_m'])
        if spacing!=scene['terrain']['sampling_grid_spacing_m']:raise ValueError('DEM spacing mismatch')
        inverse=Transformer.from_crs(scene['coordinate_reference']['projected'],4326,always_xy=True)
        queries={}
        for east in np.arange(lo[0],hi[0]+1,spacing):
            for north in np.arange(lo[1],hi[1]+1,spacing):
                lon,lat=inverse.transform(float(east),float(north));key=(round(lon,9),round(lat,9))
                if key in queries:raise ValueError('Ambiguous original grid query')
                queries[key]=(east,north)
        matched=[]
        for i,p in enumerate(samples):
            key=(p['location']['lng'],p['location']['lat'])
            if key in queries:xy[i]=queries[key];matched.append(i)
        if len(matched)!=len(queries):raise ValueError('Missing original DEM lattice samples')
        if np.max(np.linalg.norm(xy-original,axis=1))>.001:
            raise ValueError('Recovered grid displacement exceeds coordinate serialization precision')
        receipt.update(source=str(source.relative_to(root)),source_sha256=hashlib.sha256(raw).hexdigest(),
                       grid_query_count=len(queries),restored_grid_samples=len(matched),
                       unchanged_non_grid_samples=len(samples)-len(matched),
                       maximum_coordinate_restoration_m=float(np.linalg.norm(xy-original,axis=1).max()),
                       method='exact acquisition-query roundtrip match, no elevation smoothing or sample deletion')
    if restore_grid:
        original_mesh=LinearNDInterpolator(original,z).tri
        vertices=xy[original_mesh.simplices]
        # Only triangles that become exactly collinear at their original grid
        # locations disappear. Retain every other triangle and its diagonal.
        twice_area=np.linalg.det(vertices[:,:2]-vertices[:,2,None])
        keep=twice_area!=0
        terrain=RetainedLinearTerrain(xy,z,original_mesh.simplices[keep])
        receipt.update(retained_source_triangle_connectivity=True,
                       exactly_degenerate_grid_triangles_removed=int((~keep).sum()),
                       old_triangle_indices=np.flatnonzero(keep).tolist())
    else:terrain=LinearNDInterpolator(xy,z)
    receipt.update(projected_samples_sha256=hashlib.sha256(np.column_stack([xy,z]).tobytes()).hexdigest(),
                   triangles=len(terrain.tri.simplices),
                   triangulation_sha256=hashlib.sha256(terrain.tri.simplices.tobytes()).hexdigest())
    return terrain,receipt
