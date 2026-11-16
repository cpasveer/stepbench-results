import sys
sys.path.insert(0, "/home/user/stepbench-runs/tasks/pipe-L1-s1")
from build123d import import_step, Vector, Solid, Plane
import submission as S
p = import_step("/home/user/stepbench-runs/tasks/pipe-L1-s1/output.step")
print("valid", p.is_valid, "vol", round(p.volume,1))
bolts = S.TASK["poses"][0]["resolved"]["bolts"]
for b in bolts:
    pos = Vector(*b["position"]); ax = Vector(*b["axis"])
    seat = 8.0 if b["size"] == "M5" else 6.0
    base = pos + ax * seat
    hd = b["head_diameter"] / 2 + 0.5
    cyl = Solid.make_cylinder(hd, b["head_height"] + 1.0, plane=Plane(origin=base, z_dir=ax))
    vol = 0.0
    for s in p.intersect(cyl) or []:
        vol += s.volume
    print(b["id"], "head zone overlap", round(vol, 3))
