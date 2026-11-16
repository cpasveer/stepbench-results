import Part
s=Part.read("output.step")
print(len(s.Solids), s.isValid(), s.Volume, len(s.Faces))
for f in ("fixed_A.step","fixed_B.step"):
    print(f, s.common(Part.read(f)).Volume)
