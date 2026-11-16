import Part
s=Part.read('output.step').Solids[0]
for e in s.Edges:
    if e.Length<3: print('edge',round(e.Length,3),e.Vertexes[0].Point)
for i,f in enumerate(s.Faces): print(i,f.Surface.__class__.__name__,round(f.Area,1))
