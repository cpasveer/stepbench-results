import Part
s=Part.read("output.step"); f=Part.read("fixed_B.step")
c=s.common(f); print(c.Volume, c.BoundBox, len(c.Faces))
for x in c.Solids: print(x.Volume, x.BoundBox)
print([ (round(v.Point.z,6)) for v in s.Vertexes if v.Point.z>-30][:10])
for fa in f.Faces:
    print(fa.Surface.__class__.__name__, round(fa.Area,2), fa.BoundBox.ZMin, fa.BoundBox.ZMax)
