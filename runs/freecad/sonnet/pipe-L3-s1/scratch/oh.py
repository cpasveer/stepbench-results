import os, math
import FreeCAD as App, Part
import numpy as np
step=os.environ.get('STEP','output.step')
s=Part.read(step)
tris=[]
planar=[]
for fi,f in enumerate(s.Faces):
    if f.Surface.__class__.__name__=='Cylinder' and f.Surface.Radius<3.0: continue
    pts,fs=f.tessellate(0.2)
    P=np.array([[p.x,p.y,p.z] for p in pts])
    for a,b,c in fs:
        n=np.cross(P[b]-P[a],P[c]-P[a]);ar=np.linalg.norm(n)/2
        if ar<1e-9: continue
        n/=2*ar
        # orient outward using face orientation
        tris.append((fi,ar,n,(P[a]+P[b]+P[c])/3))
    if f.Surface.__class__.__name__=='Plane' and f.Area>=40:
        nrm=f.normalAt(*f.Surface.parameter(f.CenterOfMass)) if False else f.normalAt(*f.ParameterRange[0:1],*f.ParameterRange[2:3])
        planar.append((fi,f.Area,np.array([nrm.x,nrm.y,nrm.z])))
# fix triangle orientation by face orientation flag
def fixn(fi,n):
    f=s.Faces[fi]
    return -n if f.Orientation=='Reversed' else n
tris=[(fi,ar,fixn(fi,n),c) for fi,ar,n,c in tris]
# verify orientation using a sample: normal at center vs triangle normal
res=[]
cands=[('ax',np.array([0.24879564,0.59006536,0.76806484])),('-ax',-np.array([0.24879564,0.59006536,0.76806484]))]
for fi,A,nrm in planar:
    cands.append(('face%d(%.0f)'%(fi,A),-nrm))
allv=np.array([c for _,_,_,c in tris])
for name,u in cands:
    u=u/np.linalg.norm(u)
    oh=0;bed=0;worst=0;lo=(allv@u).min()
    for fi,ar,n,c in tris:
        cs=-(n@u)   # cos of angle between normal and straight down(-u)... down vector = -u ; n.(-u)
        if cs>0.9998:
            if (c@u)<lo+0.3: bed+=ar
            else: oh+=ar;worst=max(worst,90.0)
            continue
        if cs>0.7071:
            oh+=ar; worst=max(worst,90-math.degrees(math.acos(min(1,cs))))
    h=(allv@u).max()-(allv@u).min()
    print(name,'u',u.round(3),'overhang mm2 %.1f'%oh,'worst %.1f'%worst,'bed %.0f'%bed,'height %.1f'%h)
print('--- detail for ax')
u=np.array([float(x) for x in os.environ.get('UU','0.24879564,0.59006536,0.76806484').split(',')]);u/=np.linalg.norm(u)
from collections import defaultdict
d=defaultdict(float);lo=(allv@u).min()
for fi,ar,n,c in tris:
    cs=-(n@u)
    if cs>0.7071 and not (cs>0.9998 and (c@u)<lo+0.3): d[(fi,round(90-math.degrees(math.acos(min(1,cs))),0))]+=ar
for k,v in sorted(d.items()):
    if v>3: print(k,round(v,1), s.Faces[k[0]].Surface.__class__.__name__, [round(x,1) for x in s.Faces[k[0]].CenterOfMass])
