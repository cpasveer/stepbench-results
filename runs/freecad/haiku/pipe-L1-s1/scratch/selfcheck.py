import json, os
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
B = {b["id"]: b for b in T["poses"][0]["resolved"]["bolts"]}
s = Part.read(os.path.join(HERE, "output.step"))
print("solids", len(s.Solids), "valid", s.isValid(), "closed", all(sh.isClosed() for sh in s.Shells), "vol", round(s.Volume,2))
print("bbox", s.BoundBox)
fa = Part.read(os.path.join(HERE, "fixed_A.step")); fb = Part.read(os.path.join(HERE, "fixed_B.step"))
print("overlap A", round(s.common(fa).Volume,4), "overlap B", round(s.common(fb).Volume,4))
print("min dist to A", round(s.distToShape(fa)[0],4), "to B", round(s.distToShape(fb)[0],4))
# head clearance cylinders: radius head/2+0.5, from seat (0) to head top +1 along axis
for bid, b in B.items():
    p = V(*b["position"]); ax = V(*b["axis"]); hd = b["head_diameter"]; hh = b["head_height"]
    # head top plane sits at distance hh from seat (grip ends at seat)
    grip = b["grip_range"][0]
    cyl = Part.makeCylinder(hd/2+0.5, hh+1.0+ 0.0, p, ax)  # from seat to head top+1
    # also check hole through: point along axis at grip mid
    inter = s.common(cyl).Volume
    # hole check: a line along axis from seat-5 to seat+10 should be free of material
    hole = Part.makeCylinder(0.01, 30, p - ax*10, ax)
    # check head region leftover
    print(bid, "head-zone overlap vol", round(inter,4))
