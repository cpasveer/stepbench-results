import Part
s=Part.read("output.step")
print(len(s.Solids),s.Volume,s.isValid(),s.BoundBox)
A=Part.read("fixed_A.step");B=Part.read("fixed_B.step")
print(s.common(A).Volume,s.common(B).Volume)
