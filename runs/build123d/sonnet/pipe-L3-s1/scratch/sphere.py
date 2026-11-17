import numpy as np,math,sys
from build123d import *
part=import_step(sys.argv[1])
fs=part.faces()
tri=[]
for i,f in enumerate(fs):
    if f.geom_type==GeomType.CYLINDER and f.area<120: continue
    vt,tr=f.tessellate(0.15)
    for t in tr:
        a=np.array([(vt[j].X,vt[j].Y,vt[j].Z) for j in t]);n=np.cross(a[1]-a[0],a[2]-a[0]);A=np.linalg.norm(n)/2
        if A>1e-9: tri.append((i,n/np.linalg.norm(n),A,a.mean(0)))
ids=np.array([t[0] for t in tri]);ns=np.array([t[1] for t in tri]);ar=np.array([t[2] for t in tri]);cs=np.array([t[3] for t in tri])
rng=np.random.default_rng(5)
res=[]
ws=rng.normal(size=(20000,3));ws/=np.linalg.norm(ws,axis=1)[:,None]
for w in ws:
    c=ns@w
    bad=(c<-0.7071)
    # exclude bed: faces nearly flat at lowest
    z=cs@w;zmin=z.min()
    bed=(abs(z-zmin)<0.6)&(c<-0.98)
    res.append((ar[bad&~bed].sum(),(bed&bad).sum() and ar[bed].sum(),tuple(np.round(w,3))))
res.sort(key=lambda r:r[0])
for r in res[:12]:print(round(r[0],1),round(r[1],1),r[2])
