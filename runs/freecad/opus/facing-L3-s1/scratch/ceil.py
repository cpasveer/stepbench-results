import FreeCAD as App, Part
s=Part.read('output.step')
f=s.Faces[62]
for e in f.Edges:
    adj=[g for g in s.Faces if g.Area!=f.Area and any(e.isSame(x) for x in g.Edges)]
    for g in adj:
        print(e.Curve.__class__.__name__, round(e.Length,2), g.Surface.__class__.__name__, 'adj zmin/zmax', round(g.BoundBox.ZMin,2), round(g.BoundBox.ZMax,2))
