import numpy as np,sys
from build123d import *
part=import_step(sys.argv[4] if len(sys.argv)>4 else 'output.step')
fs=part.faces()
w=np.array([float(x) for x in sys.argv[1:4]]);w/=np.linalg.norm(w)
allv=np.array([[v.X,v.Y,v.Z] for v in part.vertices()])@w;zmin=allv.min()
tot=0
for i,f in enumerate(fs):
    vt,tr=f.tessellate(0.1);a_=0;mn=9
    for t in tr:
        a=np.array([(vt[j].X,vt[j].Y,vt[j].Z) for j in t]);n=np.cross(a[1]-a[0],a[2]-a[0]);A=np.linalg.norm(n)/2
        if A<1e-9:continue
        n/=np.linalg.norm(n);c=n@w
        if abs(a.mean(0)@w-zmin)<0.3 and c<-0.99: continue
        if c<-0.7071:a_+=A;mn=min(mn,c)
    if a_>0.5:print(i,f.geom_type,round(a_,1),'of',round(f.area,1),'mincos',round(mn,2),f.center());tot+=a_
print('total',tot)
