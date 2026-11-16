import json, pathlib
from build123d import *
H=pathlib.Path(__file__).parent.parent
T=json.loads((H/"task.json").read_text())
p=import_step(str(H/"output.step")).solids()[0]
print("valid",p.is_valid,"vol",p.volume, "bbox", p.bounding_box())
from collections import Counter
print(Counter(f.geom_type for f in p.faces()))
print("min edge", min(e.length for e in p.edges()), "min face", min(f.area for f in p.faces()))
for f in sorted(p.faces(), key=lambda f:f.area)[:8]: print(f.geom_type, round(f.area,2))
for n in "AB":
    fx=import_step(str(H/f"fixed_{n}.step"))
    print(n, "fixed bbox", fx.bounding_box(), "overlap", (p & fx).volume if (p&fx) else 0)
for b in T["poses"][0]["resolved"]["bolts"]:
    pos=Vector(*b["position"]); ax=Vector(*b["axis"])
    hd=b["head_diameter"]; hh=b["head_height"]
    head=extrude(Plane(origin=pos+ax*10.0, z_dir=ax)*Circle(hd/2+0.5), hh+1)
    key=extrude(Plane(origin=pos+ax*10.0, z_dir=ax)*Circle(3.0), 34)
    i1=p&head; i2=p&key
    print(b["id"], "head ov", round(i1.volume,4) if i1 else 0, "col ov", round(i2.volume,4) if i2 else 0)
