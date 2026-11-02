"""Experiment-only exact vertical rays against the imported Building mesh.

Used solely by the existing privileged locomotion observation/safety reader.
It is not a localization reference and supplies no navigation pose.
"""
from collections import defaultdict
from pathlib import Path
import numpy as np

class MeshSurface:
    def __init__(self,name,uri):
        root=Path(__file__).resolve().parents[2]
        if not uri.startswith('file://') or Path(uri[7:]).resolve()!=(root/'assets/building/Building.dae').resolve():
            raise ValueError('Building mesh must belong to this project')
        self.name=name;self.cell=.5;self.bins=defaultdict(list)
        tri=np.load(root/'assets/building/building_triangles.npz')['triangles']
        a=tri[:,0];u=tri[:,1]-a;v=tri[:,2]-a;det=u[:,0]*v[:,1]-u[:,1]*v[:,0]
        use=np.abs(det)>1e-10;self.a=a[use];self.u=u[use];self.v=v[use];self.det=det[use]
        tri=tri[use];low=np.floor(tri[:,:,:2].min(1)/self.cell).astype(int);high=np.floor(tri[:,:,:2].max(1)/self.cell).astype(int)
        for i,(lo,hi) in enumerate(zip(low,high)):
            for x in range(lo[0],hi[0]+1):
                for y in range(lo[1],hi[1]+1):self.bins[(x,y)].append(i)
        self.bins={key:np.asarray(ix,dtype=int) for key,ix in self.bins.items()}
    def intersect(self,starts):
        out=np.full(len(starts),np.inf)
        for k,p in enumerate(starts):
            ix=self.bins.get(tuple(np.floor(p[:2]/self.cell).astype(int)))
            if ix is None:continue
            q=p[:2]-self.a[ix,:2];u=self.u[ix];v=self.v[ix];d=self.det[ix]
            s=(q[:,0]*v[:,1]-q[:,1]*v[:,0])/d;t=(u[:,0]*q[:,1]-u[:,1]*q[:,0])/d
            z=self.a[ix,2]+s*u[:,2]+t*v[:,2]
            valid=(s>=-1e-9)&(t>=-1e-9)&(s+t<=1+1e-9)&(z<=p[2]+1e-9)
            if valid.any():out[k]=max(0.,float(p[2]-z[valid].max()))
        return out
