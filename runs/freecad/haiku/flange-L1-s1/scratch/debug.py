import os
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
ns = {"__file__": os.path.join(H, "submission.py")}
exec(open(os.path.join(H, "submission.py")).read(), ns)
M_W = ns["M_WORLD_FROM_A"]
A_loc = Part.read(os.path.join(H,"output.step")).transformShape(M_W.inverse())
room = Part.makeCylinder(4.75, 6.0, V(15,25.980762114,8.01), V(0,0,1))
ov = A_loc.common(room)
print("room overlap", ov.Volume, "bbox", ov.BoundBox if not ov.isNull() else None)
for s in A_loc.Solids:
    print("solid bbox", s.BoundBox)
print("pillar XY", ns["PILLAR_XY"])
