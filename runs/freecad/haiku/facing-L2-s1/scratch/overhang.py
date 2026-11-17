import os, math
import FreeCAD as App
import Part
HERE = "/home/user/stepbench-runs/tasks-freecad/facing-L2-s1"
s = Part.read(os.path.join(HERE, "output.step"))
print("solids", len(s.Solids), "valid", s.isValid(), "volume", round(s.Volume,1))
bad = []
for f in s.Faces:
    if abs(f.BoundBox.ZMin + 94) < 1e-3 and abs(f.BoundBox.ZMax + 94) < 1e-3:
        continue  # bed contact
    u0, u1, v0, v1 = f.ParameterRange
    n = f.normalAt((u0+u1)/2, (v0+v1)/2)
    if f.Orientation == "Reversed":
        n = n * -1
    if n.z < -math.cos(math.radians(45)) - 1e-9:
        bad.append((round(f.Area,2), round(f.BoundBox.ZMin,2), round(f.BoundBox.ZMax,2), round(n.z,3)))
print("downward faces steeper than 45 deg:", len(bad), bad[:10])
print("faces", len(s.Faces))
