import Part
for f in ["fixed_A.step","fixed_B.step"]:
    s=Part.read(f); print(f, s.BoundBox, len(s.Solids), s.Volume)
    for fc in s.Faces:
        print("  ", fc.Surface.__class__.__name__, round(fc.Area,1), fc.BoundBox.Center)
