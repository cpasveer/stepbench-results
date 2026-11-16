import json,sys
from build123d import *
t=json.load(open("task.json"))
p=import_step("output.step")
print("solids",len(p.solids()),p.is_valid,p.volume,p.bounding_box().min,p.bounding_box().max)
for n in "AB":
    f=import_step(f"fixed_{n}.step"); print(n,"overlap",(p&f).volume if (p&f) else 0)
for b in t["poses"][0]["resolved"]["bolts"]:
    x,y,z=b["position"]; ax=b["axis"]
    # probe: material at axis? and ring
    c=Pos(x,y,z+ax[2]*5)*Cylinder(b["clearance_diameter_range"][0]/2-0.05,30)
    r=Pos(x,y,z+ax[2]*1)*(Cylinder(b["head_diameter"]/2,0.5)-Cylinder(b["clearance_diameter_range"][1]/2,0.5))
    i=p&r
    tool=Pos(x,y,z+ax[2]*(40))*Cylinder(b["head_diameter"]/2+0.5,60)
    grip=3.0 if b["size"]=="M4" else 3.8
    tool=Pos(x,y,z+ax[2]*(grip+0.01+17.5))*Cylinder(b["head_diameter"]/2+0.5,35)
    j=p&tool
    print(b["id"],"hole-overlap",(p&c).volume if (p&c) else 0,"ring frac",i.volume/r.volume,"tool",j.volume if j else 0)
fs=sorted(p.faces(),key=lambda f:f.area)[:6]
print([round(f.area,3) for f in fs]); print(sorted(round(e.length,3) for e in p.edges())[:6])
