import sys; sys.path.insert(0,'.')
import submission as s
from submission import *
import itertools
for du,dv in itertools.product([-4,-2,0,2,4],[-4,-2,0,2,4]):
    s.QU=du; s.QV=dv
    r,D=s.riser()
    tot=0
    for k in ["B1","B2","B3","B4"]:
        b=BOLTS[k]; p=loc(b["position"]); a=dirl(b["axis"])
        c=Plane(origin=V(p+a*6),z_dir=V(a))*Cylinder(4,5,align=(Align.CENTER,Align.CENTER,Align.MIN))
        tot+=(r&c).volume
    print(du,dv,round(tot,2), round(float(D@s.BN),2))
