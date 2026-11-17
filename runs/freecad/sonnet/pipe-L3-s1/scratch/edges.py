import os, math
import FreeCAD as App, Part
import numpy as np
V=App.Vector
s=Part.read(os.environ.get('STEP','scratch/v.step'))
sol=s.Solids[0]
# map edges to faces
emap={}
for fi,f in enumerate(sol.Faces):
    for e in f.Edges:
        for ei,ee in enumerate(sol.Edges):
            pass
import collections
ed=collections.defaultdict(list)
for fi,f in enumerate(sol.Faces):
    for e in f.Edges:
        ed[sol.Edges.index(e) if False else None].append(fi)
# use hash-based approach
idx={}
for i,e in enumerate(sol.Edges): idx[e.hashCode()]=i
ef=collections.defaultdict(list)
for fi,f in enumerate(sol.Faces):
    for e in f.Edges:
        ef[idx[e.hashCode()]].append(fi)
out=[]
for ei,fl in ef.items():
    if len(fl)!=2: continue
    e=sol.Edges[ei]
    if e.Length<0.01: continue
    mid=e.valueAt((e.FirstParameter+e.LastParameter)/2)
    ns=[]
    for fi in fl:
        f=sol.Faces[fi]
        u,v=f.Surface.parameter(mid)
        nrm=f.Surface.normal(u,v)
        if f.Orientation=='Reversed': nrm=-nrm
        ns.append(nrm)
    c=ns[0].dot(ns[1]);c=max(-1,min(1,c))
    tn=math.degrees(math.acos(c))
    if tn<0.5: continue
    # convex?
    p=mid+(ns[0]+ns[1])*0.01
    inside=sol.isInside(p,1e-6,True)
    convex=not inside
    ang=180-tn
    flag=''
    if convex and ang<5: flag='THIN'
    if (not convex) and ang<45: flag='SHALLOW'
    if tn<2: flag='CREASE'
    if flag or os.environ.get('ALL'):
        out.append((flag,round(ang,1),'convex' if convex else 'concave',round(e.Length,1),[round(x,1) for x in (mid.x,mid.y,mid.z)],fl))
for o in out: print(o)
print('faces',len(sol.Faces),'edges',len(sol.Edges))
# small faces / short edges
for fi,f in enumerate(sol.Faces):
    if f.Area<2: print('smallface',fi,round(f.Area,2),[round(x,1) for x in f.CenterOfMass])
for ei,e in enumerate(sol.Edges):
    if e.Length<0.6: print('shortedge',ei,round(e.Length,2),[round(x,1) for x in e.CenterOfMass])
