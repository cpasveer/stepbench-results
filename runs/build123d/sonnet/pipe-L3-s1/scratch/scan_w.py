import numpy as np,math,sys
from build123d import *
part=import_step('output.step')
N=np.array([0.3885173161164798,0.9214414224885092,0]);E2=np.array([0,0,1.0])
fs=part.faces()
tri=[]
for i,f in enumerate(fs):
    if f.geom_type==GeomType.CYLINDER and f.area<120: continue
    vt,tr=f.tessellate(0.1)
    for t in tr:
        a=np.array([(vt[j].X,vt[j].Y,vt[j].Z) for j in t]);n=np.cross(a[1]-a[0],a[2]-a[0]);A=np.linalg.norm(n)/2
        if A>1e-9: tri.append((i,n/np.linalg.norm(n),A,a.mean(0)))
ids=np.array([t[0] for t in tri]);ns=np.array([t[1] for t in tri]);ar=np.array([t[2] for t in tri]);cs=np.array([t[3] for t in tri])
for g in (-12,-6,0,6,10,12,15,20):
    w=-E2*math.cos(math.radians(g))+N*math.sin(math.radians(g))
    c=ns@w;z=cs@w;zmin=z.min()
    bed=(abs(z-zmin)<0.6)&(c<-0.98)
    m=(c<-0.7071)&(~bed)
    d={}
    for i,a in zip(ids[m],ar[m]): d[i]=d.get(i,0)+a
    print(g,round(ar[m].sum(),1),[(i,round(a)) for i,a in sorted(d.items(),key=lambda k:-k[1])[:5]])
