import json, os
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
B = {b["id"]: b for b in T["poses"][0]["resolved"]["bolts"]}
s = Part.read(os.path.join(HERE, "output.step"))
b = B["A1"]; p = V(*b["position"]); ax = V(*b["axis"])
tests = {
 "head r4.75 ax[0,13]": Part.makeCylinder(4.75, 13, p, ax),
 "head r4.75 ax[7,14]": Part.makeCylinder(4.75, 7, p+ax*7, ax),
 "head r4.25 ax[8,13]": Part.makeCylinder(4.25, 5, p+ax*8, ax),
 "shank r2.75 ax[-2,8]": Part.makeCylinder(2.75, 10, p-ax*2, ax),
 "shank r3 ax[-5,8]": Part.makeCylinder(3.0, 13, p-ax*5, ax),
 "head r4.75 ax[7.5,14] (0.5 margin)": Part.makeCylinder(4.75, 6.5, p+ax*7.5, ax),
}
for k, c in tests.items():
    print(k, "overlap", round(s.common(c).Volume, 4))
# where is it: sample the common region's bounding box
com = s.common(tests["head r4.75 ax[0,13]"])
print("common bbox", com.BoundBox if com.Volume > 0 else None)
