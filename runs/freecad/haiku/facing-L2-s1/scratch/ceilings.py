import os
import FreeCAD as App
import Part

HERE = "/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
s = Part.read(os.path.join(HERE, "output.step"))
tot = 0.0
rows = []
for f in s.Faces:
    # downward-facing (outward normal has negative z), not on the bed (z=-94)
    u0, u1, v0, v1 = f.ParameterRange
    n = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
    if f.Surface.__class__.__name__ == "Plane" and n.z < -0.707 and abs(f.BoundBox.ZMin + 94) > 1e-3:
        rows.append((round(f.Area, 2), round(f.BoundBox.ZMin, 2), round(f.BoundBox.ZMax, 2), tuple(round(c, 1) for c in f.CenterOfMass)))
        tot += f.Area
for r in sorted(rows, reverse=True):
    print("ceiling face area %.2f  z %.2f..%.2f  centre %s" % r)
print("total downward planar (non-bed) area %.2f" % tot)
