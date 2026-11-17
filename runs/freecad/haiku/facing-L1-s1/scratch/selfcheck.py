import os, FreeCAD as App, Part
H = "/home/user/stepbench-runs/tasks-freecad/facing-L1-s1"
out = Part.read(os.path.join(H, "output.step"))
A = Part.read(os.path.join(H, "fixed_A.step"))
B = Part.read(os.path.join(H, "fixed_B.step"))
print("solids", len(out.Solids), "valid", out.isValid())
print("overlap A", out.common(A).Volume, "overlap B", out.common(B).Volume)
print("bbox", out.BoundBox)
print("A bbox", A.BoundBox, "B bbox", B.BoundBox)
