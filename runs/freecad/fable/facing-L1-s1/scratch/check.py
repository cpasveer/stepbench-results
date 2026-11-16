import Part
s=Part.read("output.step");print(s.isValid(),len(s.Solids),len(s.Faces),s.Volume,s.BoundBox)
for n in "AB":
    f=Part.read("fixed_%s.step"%n);print(n,s.common(f).Volume)
for f in s.Faces: print(f.Surface.__class__.__name__, round(f.Area,2))
