import sys,numpy as np,math
from build123d import *
from collections import defaultdict
part=import_step(sys.argv[1]);w=np.array([float(x) for x in sys.argv[2:5]]);w/=np.linalg.norm(w)
emap=defaultdict(list);ekeys={}
for f in part.faces():
    for e in f.edges():
        c=e.center();k=(round(c.X,2),round(c.Y,2),round(c.Z,2),round(e.length,2))
        emap[k].append(f);ekeys[k]=e
tot=0
for k,fs in emap.items():
    if len(fs)!=2: continue
    e=ekeys[k]
    if e.geom_type not in (GeomType.LINE,): continue
    p=e.position_at(.5)
    n=[np.array([f.normal_at(p).X,f.normal_at(p).Y,f.normal_at(p).Z]) for f in fs]
    if n[0]@w<-0.05 and n[1]@w<-0.05:
        t=e.tangent_at(.5);t=np.array([t.X,t.Y,t.Z]);ang=math.degrees(math.acos(min(1,abs(t@w))))
        if ang>60:
            print('keel ang',round(ang,1),'len',round(e.length,1),np.round([p.X,p.Y,p.Z],1),fs[0].geom_type,fs[1].geom_type,'c',np.round([n[0]@w,n[1]@w],2));tot+=e.length
print('total',tot)
