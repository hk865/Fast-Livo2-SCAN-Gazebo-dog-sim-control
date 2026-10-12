"""URDF-bounded joint-reference return geometry for the physical controller."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


class JointGeometry:
    def __init__(self,urdf,height,com_x=0.):
        root=ET.fromstring(urdf)
        self.joints={j.attrib['name']:j for j in root.findall('joint')}
        self.names=[f'{leg}_{joint}_joint' for leg in ['lf','rf','lh','rh'] for joint in ['hip','upper_leg','lower_leg']]
        self.lower=np.array([float(self.joints[n].find('limit').attrib['lower']) for n in self.names])
        self.upper=np.array([float(self.joints[n].find('limit').attrib['upper']) for n in self.names])
        self.velocity=np.array([float(self.joints[n].find('limit').attrib['velocity']) for n in self.names])
        self.nominal_feet=[]
        for leg in ['lf','rf','lh','rh']:
            origins=[np.fromstring(self.joints[f'{leg}_{suffix}_joint'].find('origin').attrib['xyz'],sep=' ')
                     for suffix in ['hip','upper_leg','lower_leg','foot']]
            # Same nominal geometry convention used by CHAMP BodyController.
            z=sum(o[2] for o in origins)
            self.nominal_feet.append([origins[0][0]+origins[1][0]+com_x,
                origins[0][1]+origins[1][1],z+np.clip(-(z+height),0,-z*.65)])
        self.nominal_feet=np.array(self.nominal_feet)

    def valid(self,q):
        q=np.asarray(q)
        return q.shape==(12,) and np.all(np.isfinite(q)) and np.all(q>=self.lower-1e-7) and np.all(q<=self.upper+1e-7)

    def feet(self,q):
        if not self.valid(q):raise ValueError('joint positions outside URDF limits')
        angles=dict(zip(self.names,q));feet=[]
        for leg in ['lf','rf','lh','rh']:
            pos=np.zeros(3);rotation=Rotation.identity()
            for suffix in ['hip','upper_leg','lower_leg','foot']:
                name=f'{leg}_{suffix}_joint';joint=self.joints[name];origin=joint.find('origin')
                pos+=rotation.apply(np.fromstring(origin.attrib['xyz'],sep=' '))
                rotation=rotation*Rotation.from_euler('xyz',np.fromstring(origin.attrib['rpy'],sep=' '))
                if name in angles:rotation=rotation*Rotation.from_rotvec(np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')*angles[name])
            feet.append(pos)
        return np.array(feet)

    def is_nominal(self,q):
        return self.valid(q) and float(np.max(np.linalg.norm(self.feet(q)-self.nominal_feet,axis=1)))<=2e-5


class StopReturn:
    """C0 at entry; C2 inside/at end. Does not claim incoming gait velocity continuity."""
    def __init__(self,q0,nominal,geometry,stamp):
        self.q0=np.array(q0,dtype=float);self.nominal=np.array(nominal,dtype=float)
        if not geometry.valid(self.q0) or not geometry.is_nominal(self.nominal):raise ValueError('invalid stop endpoints')
        self.delta=self.nominal-self.q0;self.start=float(stamp);self.last=float(stamp)
        self.vmax=np.minimum(3.,geometry.velocity)
        # Exact extrema for quintic smoothstep: ds=1.875, |dds|=10/sqrt(3).
        self.duration=max(.30,float(np.max(1.875*np.abs(self.delta)/self.vmax)),
            math.sqrt(float(np.max(np.abs(self.delta)))*(10/math.sqrt(3))/20.))
        self.geometry=geometry

    def evaluate(self,stamp):
        if not math.isfinite(stamp) or stamp<self.last-1e-9:raise ValueError('backwards or invalid simulation time')
        self.last=float(stamp);s=float(np.clip((stamp-self.start)/self.duration,0.,1.))
        blend=10*s**3-15*s**4+6*s**5
        q=self.q0+self.delta*blend
        v=self.delta*(30*s*s-60*s**3+30*s**4)/self.duration
        acc=self.delta*(60*s-180*s*s+120*s**3)/self.duration**2
        if not self.geometry.valid(q) or np.any(np.abs(v)>self.vmax+1e-7) or np.any(np.abs(acc)>20.+1e-6):
            raise ValueError('stop reference bounds violated')
        return q,v,acc,s>=1.
