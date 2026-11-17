import Part, FreeCAD as App
for f in ["fixed_A.step","fixed_B.step"]:
    s=Part.read(f); print(f, s.BoundBox, len(s.Faces))
