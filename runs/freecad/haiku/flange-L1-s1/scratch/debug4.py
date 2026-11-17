import os
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
ns = {"__file__": os.path.join(H, "submission.py")}
exec(open(os.path.join(H, "submission.py")).read(), ns)
W = Part.read(os.path.join(H,"output.step"))
A1 = W.transformShape(ns["M_WORLD_FROM_A"].inverse())
A2 = Part.read(os.path.join(H,"output.step")).transformShape(ns["M_WORLD_FROM_A"].inverse())
for nm,S in [("A1",A1),("A2",A2)]:
    print(nm, "in hole(15,26,4)", S.isInside(V(15,25.98,4),0.001,True), "in (15,26,10)", S.isInside(V(15,25.98,10),0.001,True), "in (15,26,-0.5)", S.isInside(V(15,25.98,-0.5),0.001,True))
    room = Part.makeCylinder(4.75, 5.99, V(15,25.980762114,8.01), V(0,0,1))
    print(nm, "room overlap", round(S.common(room).Volume,4))
print("pad top z via pad at (0,0):", A1.isInside(V(0,0,7.9),0.001,True), A1.isInside(V(0,0,8.1),0.001,True))
