import sys; sys.path.insert(0,'.')
from submission import *
a=pl2loc(*P0); d=unit(S_END-a); print("d",np.round(d,3))
ex=np.array([1.0,0,0]); p1=unit(ex-d*(ex@d)); p2=np.cross(d,p1)
fn=[unit(p1+p2),unit(p2-p1),unit(-p1-p2),unit(p1-p2)]
fl=[np.array([0,math.sin(ANG),math.cos(ANG)]),np.array([0,-math.sin(ANG),math.cos(ANG)])]
for f in fn:
    print(np.round(f,2),[round(math.degrees(math.acos(f@g)),1) for g in fl])
