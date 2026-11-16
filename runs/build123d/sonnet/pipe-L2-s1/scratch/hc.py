import sys; sys.path.insert(0,'.')
from submission import *
part=build()
for k,b in BOLTS.items():
    p=loc(b["position"]); a=dirl(b["axis"])
    g=6.0
    c=Plane(origin=V(p+a*g),z_dir=V(a))*Cylinder(b["head_diameter"]/2+0.5,b["head_height"]+1,align=(Align.CENTER,Align.CENTER,Align.MIN))
    t=Plane(origin=V(p-a*3),z_dir=V(a))*Cylinder(3.2,160,align=(Align.CENTER,Align.CENTER,Align.MIN))
    f=lambda r: round(r.volume,2) if r is not None else 0
    print(k,f(part&c),f(part&t)-0)
b=BOLTS["B1"]; p=loc(b["position"]); a=dirl(b["axis"])
t=Plane(origin=V(p+a*8),z_dir=V(a))*Cylinder(3.2,160,align=(Align.CENTER,Align.CENTER,Align.MIN))
r=part&t
for v in r.vertices():
    q=np.array([v.X,v.Y,v.Z])-BO
    print(round(q@BU,1),round(q@BV,1),round(q@BN,1))
