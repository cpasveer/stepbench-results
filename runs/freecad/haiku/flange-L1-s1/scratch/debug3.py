import os
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
ns = {"__file__": os.path.join(H, "submission.py")}
exec(open(os.path.join(H, "submission.py")).read(), ns)
A_loc = Part.read(os.path.join(H,"output.step")).transformShape(ns["M_WORLD_FROM_A"].inverse())
for x,y in [(30.0,0.0),(15.0,25.980762114),(-15.0,25.980762114),(-30.0,0.0),(-15.0,-25.980762114),(15.0,-25.980762114)]:
    room = Part.makeCylinder(4.75, 5.99, V(x,y,8.01), V(0,0,1))
    print("bolt", x, y, "overlap", round(A_loc.common(room).Volume,4))
