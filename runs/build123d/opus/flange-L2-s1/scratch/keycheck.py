import json, math, sys
from build123d import *
T=json.load(open("task.json"))["poses"][0]["resolved"]["bolts"]
part=import_step("output.step")
grip={"M5":8.0,"M6":8.0}
key={"M5":(4,25,70),"M6":(5,28,80)}
for b in T:
    ax=Vector(*b["axis"]).normalized(); p=Vector(*b["position"])
    s,short,long_=key[b["size"]]
    r=s*1.15/2
    top=p+ax*(grip[b["size"]]+b["head_height"])
    pl=Plane(origin=top, z_dir=ax)
    # head + clearance
    head=pl*Location((0,0,-b["head_height"]))*Cylinder(b["head_diameter"]/2+0.5,b["head_height"]+1,align=(Align.CENTER,Align.CENTER,Align.MIN))
    hv=(head&part).volume
    legz=short-r
    sl=pl*Cylinder(r,legz,align=(Align.CENTER,Align.CENTER,Align.MIN))
    sv=(sl&part).volume
    free=[]
    for ang in range(0,360,10):
        ll=pl*Location((0,0,legz))*Rotation(0,0,ang)*Rotation(0,90,0)*Cylinder(r,long_,align=(Align.CENTER,Align.CENTER,Align.MIN))
        free.append((ll&part).volume<1e-3)
    # longest run of free
    best=0
    for i in range(36):
        n=0
        while n<36 and free[(i+n)%36]: n+=1
        best=max(best,n)
    print(b["id"],"head",round(hv,3),"short",round(sv,3),"free arc deg",(best-1)*10 if best else 0)
