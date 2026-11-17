import json
import FreeCAD as App, Part
V = App.Vector
TD = "/home/user/stepbench-runs/tasks-freecad/pipe-L2-s1"
s = Part.read(TD + "/output.step").Solids[0]
T = json.load(open(TD + "/task.json"))
PL = T["poses"][0]["placements"]
OB = V(*PL["B"]["origin"]); R = PL["B"]["rotation"]
XB = V(R[0][0], R[1][0], R[2][0]); YB = V(R[0][1], R[1][1], R[2][1]); ZB = V(R[0][2], R[1][2], R[2][2])
M = App.Matrix(XB.x, YB.x, ZB.x, OB.x, XB.y, YB.y, ZB.y, OB.y, XB.z, YB.z, ZB.z, OB.z, 0, 0, 0, 1)
slab = Part.makeBox(200, 200, 0.01, V(-100, -100, 0)).transformShape(M)
sec = s.common(slab)
print("seat area on B face (mm2):", round(sec.Volume / 0.01, 3))
inset = Part.makeBox(36, 36, 0.01, V(-18, -18, 0)).transformShape(M)
outside = slab.cut(inset)
print("seat area beyond 1 mm inset (mm2):", round(s.common(outside).Volume / 0.01, 4))
