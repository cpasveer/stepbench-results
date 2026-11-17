import numpy as np,sys
from build123d import *
part=import_step('output.step')
tri=[]
for i,f in enumerate(part.faces()):
    vt,tr=f.tessellate(0.2)
    for t in tr:
        a=np.array([(vt[j].X,vt[j].Y,vt[j].Z) for j in t]);n=np.cross(a[1]-a[0],a[2]-a[0]);A=np.linalg.norm(n)/2
        if A>0: tri.append((i,n/np.linalg.norm(n)*1,A,a.mean(0)))
ids=np.array([t[0] for t in tri]);ns=np.array([t[1] for t in tri]);ar=np.array([t[2] for t in tri])
rng=np.random.default_rng(1)
res=[]
for w in rng.normal(size=(6000,3)):
    w/=np.linalg.norm(w);c=ns@w
    res.append((ar[c<-0.7071].sum(),tuple(w)))
res.sort()
print(res[:3])
w=np.array(res[0][1]);c=ns@w
d={}
for i,x,a in zip(ids,c,ar):
    if x<-0.7071:d[i]=d.get(i,0)+a
fs=part.faces()
for i,a in sorted(d.items(),key=lambda k:-k[1])[:8]:
    f=fs[i];print(i,f.geom_type,round(a,1),round(f.area,1),f.center())
