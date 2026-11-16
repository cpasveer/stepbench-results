import Part
for f in ["fixed_A.step","fixed_B.step"]:
    s=Part.read(f)
    print(f, len(s.Solids), s.BoundBox, s.Volume)
    for so in s.Solids: print("  ", so.BoundBox, so.Volume, len(so.Faces))
