import os, math, collections
import FreeCAD as App, Part
import numpy as np
step=os.environ.get('STEP','output.step')
u=np.array([float(x) for x in os.environ.get('UU','0.24879564,0.59006536,0.76806484').split(',')]);u/=np.linalg.norm(u)
sol=Part.read(step).Solids[0]
idx={e.hashCode():i for i,e in enumerate(sol.Edges)}
ef=collections.defaultdict(list)
for fi,f in enumerate(sol.Faces):
    for e in f.Edges: ef[idx[e.hashCode()]].append(fi)
lo=min((np.array([v.X,v.Y,v.Z])@u) for v in sol.Vertexes)
tot=0
for ei,fl in ef.items():
    if len(fl)!=2: continue
    e=sol.Edges[ei]
    mid=e.valueAt((e.FirstParameter+e.LastParameter)/2)
    ns=[]
    for fi in fl:
        f=sol.Faces[fi]
        uu,vv=f.Surface.parameter(mid);nm=f.Surface.normal(uu,vv)
        if f.Orientation=='Reversed': nm=-nm
        ns.append(np.array([nm.x,nm.y,nm.z]))
    if not all((n@u)<-0.05 for n in ns): continue   # both faces face downward
    p0=e.valueAt(e.FirstParameter);p1=e.valueAt(e.LastParameter)
    d=np.array([p1.x-p0.x,p1.y-p0.y,p1.z-p0.z]);L=np.linalg.norm(d)
    if L<0.5: continue
    d/=L
    ang=math.degrees(math.acos(min(1,abs(d@u))))
    lvl=np.array([mid.x,mid.y,mid.z])@u-lo
    if lvl<0.3: continue
    # convex check
    pp=mid+App.Vector(*((ns[0]+ns[1])*0.02))
    convex=not sol.isInside(pp,1e-6,True)
    print('edge len %.1f ang %.1f level %.1f convex %s faces %s'%(L,ang,lvl,convex,fl))
    if ang>63.4 and convex: tot+=L
print('keel length over limit',round(tot,1))
