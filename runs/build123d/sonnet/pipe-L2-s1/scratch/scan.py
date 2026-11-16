import sys; sys.path.insert(0,'.')
import submission as s
from submission import *
import itertools
def tubes(arm):
    tot=0
    for k in ["B1","B2","B3","B4"]:
        b=BOLTS[k]; p=loc(b["position"]); a=dirl(b["axis"])
        t=Plane(origin=V(p-a*3),z_dir=V(a))*Cylinder(2.9,160,align=(Align.CENTER,Align.CENTER,Align.MIN))
        r=arm&t; tot+=r.volume if r is not None else 0
        c=Plane(origin=V(p+a*6),z_dir=V(a))*Cylinder(4,6,align=(Align.CENTER,Align.CENTER,Align.MIN))
        r=arm&c; tot+=r.volume if r is not None else 0
    return tot
for sx,sz,u0,v0 in itertools.product([14,17,20],[38,42],[-5,-8],[2,3.5]):
    s.SX=sx; s.SZ=sz; s.S_END=np.array([sx,0,sz]); s.P0=(u0,v0,s.P0[2])
    a=s.pl2loc(u0,v0,0.0); d=unit(s.S_END-a)
    g=math.degrees(math.acos(abs(d[0])))
    ar=s.arm()
    print(sx,sz,u0,v0,"gamma",round(g,1),"tube",round(tubes(ar),1))
