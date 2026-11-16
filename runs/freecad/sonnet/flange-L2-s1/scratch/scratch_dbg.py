import Part
s=Part.read("output.step")
for x in s.Solids: print(x.Volume, x.BoundBox)
