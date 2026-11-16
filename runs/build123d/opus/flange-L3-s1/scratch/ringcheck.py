import json, math
from build123d import *
p=import_step("output.step")
T=json.load(open("task.json"))
for b in T["poses"][0]["resolved"]["bolts"]:
    ax=Vector(*b["axis"]); pos=Vector(*b["position"])
    ro=b["head_diameter"]/2; ri=b["clearance_diameter_range"][1]/2
    pl=Plane(origin=pos, z_dir=ax)
    ring=pl*(Cylinder(ro,0.3,align=(Align.CENTER,Align.CENTER,Align.MIN))-Cylinder(ri,0.3,align=(Align.CENTER,Align.CENTER,Align.MIN)))
    r=p&ring; v=0 if r is None else r.volume
    print(b["id"], round(v/ring.volume,3))
