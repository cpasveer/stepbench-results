import Part
s=Part.read("output.step"); print(len(s.Solids), s.isValid(), s.Volume, len(s.Faces))
for n in ["fixed_A.step","fixed_B.step"]:
    print(n, s.common(Part.read(n)).Volume)
