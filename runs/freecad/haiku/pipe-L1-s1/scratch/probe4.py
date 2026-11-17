import json, os
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
B = {b["id"]: b for b in T["poses"][0]["resolved"]["bolts"]}
s = Part.read(os.path.join(HERE, "output.step"))
print("valid", s.isValid(), "solids", len(s.Solids), "vol", round(s.Volume,2))
for bid in ["A1","A2","A3","A4"]:
    b = B[bid]; p = V(*b["position"]); ax = V(*b["axis"])
    res = []
    for start, rad in [(7.9, 4.75), (7.95, 4.75), (8.0, 4.75), (7.95, 4.25)]:
        c = Part.makeCylinder(rad, 14 - start, p + ax*start, ax)
        res.append((start, rad, round(s.common(c).Volume, 5)))
    print(bid, res)
