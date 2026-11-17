import numpy as np,sys
from build123d import *
part=import_step(sys.argv[1] if len(sys.argv)>1 else 'output.step')
tri=[]
fs=part.faces()
for i,f in enumerate(fs):
    vt,tr=f.tessellate(0.1)
    for t in tr:
        a=np.array([(vt[j].X,vt[j].Y,vt[j].Z) for j in t]);n=np.cross(a[1]-a[0],a[2]-a[0]);A=np.linalg.norm(n)/2
        if A>1e-9: tri.append((i,n/np.linalg.norm(n),A))
cen=np.array([ (f.center().X,f.center().Y,f.center().Z) for f in fs])
ids=np.array([t[0] for t in tri]);ns=np.array([t[1] for t in tri]);ar=np.array([t[2] for t in tri])
rng=np.random.default_rng(1)
zs=None
def score(w,full=False):
    c=ns@w
    # vertices lowest
    z=cen@w
    allv=np.array([[v.X,v.Y,v.Z] for v in part.vertices()])@w
    zmin=allv.min()
    bed=np.array([abs(cen[i]@w-zmin)<0.3 and True for i in ids])
    m=(c<-0.7071)&(~bed)
    return ar[m].sum()
best=min(((score(w/np.linalg.norm(w)),tuple(w/np.linalg.norm(w))) for w in rng.normal(size=(3000,3))))
# refine
w=np.array(best[1]);s=best[0]
for it in range(600):
    w2=w+rng.normal(size=3)*0.03;w2/=np.linalg.norm(w2);s2=score(w2)
    if s2<=s:w,s=w2,s2
print('best',s,np.round(w,3))
c=ns@w;d={}
for i,x,a in zip(ids,c,ar):
    if x<-0.7071:d[i]=d.get(i,0)+a
for i,a in sorted(d.items(),key=lambda k:-k[1])[:10]:
    f=fs[i];print(i,f.geom_type,round(a,1),round(f.area,1),f.center(), 'normalz',round(float(np.min(c[ids==i])),2))
