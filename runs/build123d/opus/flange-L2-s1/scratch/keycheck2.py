import json, math, sys
from build123d import *
T=json.load(open("task.json"))["poses"][0]["resolved"]["bolts"]
part=import_step("output.step")
only=sys.argv[1:] 
key={"M5":(4,25,70),"M6":(5,28,80)}
for b in T:
    if only and b["id"] not in only: continue
    ax=Vector(*b["axis"]).normalized(); p=Vector(*b["position"])
    s,short,long_=key[b["size"]]
    r=s*1.15/2
    top=p+ax*(8.0+b["head_height"])
    res=[]
    for tilt in (0,10,20):
      for tdir in (range(0,360,45) if tilt else [0]):
        pl=Plane(origin=top, z_dir=ax)*Rotation(0,0,tdir)*Rotation(tilt,0,0)
        legz=short-r
        sl=pl*Cylinder(r,legz,align=(Align.CENTER,Align.CENTER,Align.MIN))
        if (sl&part).volume>1e-3: continue
        free=[]
        for ang in range(0,360,10):
            ll=pl*Location((0,0,legz))*Rotation(0,0,ang)*Rotation(0,90,0)*Cylinder(r,long_,align=(Align.CENTER,Align.CENTER,Align.MIN))
            free.append((ll&part).volume<1e-3)
        best=0
        for i in range(36):
            n=0
            while n<36 and free[(i+n)%36]: n+=1
            best=max(best,n)
        res.append(((best-1)*10 if best else 0,tilt,tdir))
        if tilt==0: break
    print(b["id"], max(res) if res else None, "tilt0:", [x for x in res if x[1]==0])
