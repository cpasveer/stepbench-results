import json, os
import FreeCAD as App, Part
V = App.Vector
HERE = "/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1"
T = json.load(open(os.path.join(HERE, "task.json")))
B = {b["id"]: b for b in T["poses"][0]["resolved"]["bolts"]}
s = Part.read(os.path.join(HERE, "output.step"))
fb = Part.read(os.path.join(HERE, "fixed_B.step"))
print("bbox out", s.BoundBox)
for bid in ["B1","B4"]:
    b = B[bid]; p = V(*b["position"]); ax = V(*b["axis"])
    for d in [0,2,4,6,8,10,12]:
        q = p + ax*d
        print(bid, "d", d, "inside out", s.isInside(q,1e-6,True), "inside fixedB", fb.isInside(q,1e-6,True))
# B face plane check: distance of bolt pos from the output's nearest face
print("dist B1 to output", round(s.distToShape(Part.makeSphere(0.01, V(*B['B1']['position'])))[0],3))
