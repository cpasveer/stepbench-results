import os
import FreeCAD as App, Part
V = App.Vector
H = "/home/user/stepbench-runs/tasks-freecad/flange-L1-s1"
def perp(d):
    d = V(d); d.normalize()
    t = V(1,0,0) if abs(d.x) < 0.9 else V(0,1,0)
    e1 = d.cross(t); e1.normalize(); e2 = d.cross(e1); e2.normalize()
    return d, e1, e2
p = V(15.0, 25.980762114, 0)
d = V(0,0,1)
dn, e1, e2 = perp(d)
print("dn", dn, "p", p)
base = p + dn*(8.0+0.01)
print("base", base)
room = Part.makeCylinder(4.75, 6.0-0.01, base, dn)
print("room bbox", room.BoundBox)
A_loc = Part.read(os.path.join(H,"output.step"))
import FreeCAD
M = App.Matrix(); 
