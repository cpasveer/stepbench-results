import json, os
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
B = {b["id"]: b for b in T["poses"][0]["resolved"]["bolts"]}
s = Part.read(os.path.join(HERE, "output.step"))
for bid in ["A1","A2","A3","A4"]:
    b = B[bid]; p = V(*b["position"]); ax = V(*b["axis"])
    row = []
    for start in [7.0, 7.5, 7.8, 8.0]:
        for rad in [4.25, 4.75]:
            c = Part.makeCylinder(rad, 14 - start, p + ax*start, ax)
            row.append((start, rad, round(s.common(c).Volume, 4)))
    print(bid, row)
