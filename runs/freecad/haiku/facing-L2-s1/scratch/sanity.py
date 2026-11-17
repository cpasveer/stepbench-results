import json, math, os
import FreeCAD as App
import Part

HERE = "/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
V = App.Vector
task = json.load(open(os.path.join(HERE, "task.json")))
res = task["poses"][0]["resolved"]
bolts = {b["id"]: b for b in res["bolts"]}

out = Part.read(os.path.join(HERE, "output.step"))
print("solids", len(out.Solids), "valid", out.isValid(), "closed", all(s.isClosed() for s in out.Solids) if out.Solids else None)
print("volume %.1f mm3  area %.1f" % (out.Volume, out.Area))
bb = out.BoundBox
print("bbox", round(bb.XMin,2), round(bb.XMax,2), round(bb.YMin,2), round(bb.YMax,2), round(bb.ZMin,2), round(bb.ZMax,2))
print("faces", len(out.Faces), "edges", len(out.Edges))
mine_min_edge = min(e.Length for e in out.Edges)
print("min edge %.3f" % mine_min_edge)
print("min face area %.3f" % min(f.Area for f in out.Faces))

fa = Part.read(os.path.join(HERE, "fixed_A.step"))
fb = Part.read(os.path.join(HERE, "fixed_B.step"))
print("overlap A %.4f  B %.4f" % (out.common(fa).Volume, out.common(fb).Volume))
print("distance to A %.4f  to B %.4f" % (out.distToShape(fa)[0], out.distToShape(fb)[0]))

# bearing and seat checks on each bolt
for bid, b in sorted(bolts.items()):
    p = V(*b["position"])
    ax = V(*b["axis"])
    # plane through the bolt hole region: probe the material around the bolt along the axis
    hole_d = 4.5 if b["size"] == "M4" else 5.5
    # cylinder of the hole must be empty
    hole = Part.makeCylinder(hole_d/2 - 0.2, 60, p - ax*30, ax)
    head_r = b["head_diameter"]/2 + 0.5
    print(bid, "hole free:", round(out.common(hole).Volume, 4))
