import Part, FreeCAD as App
f=Part.read("fixed_A.step"); print(f.BoundBox, f.Volume, len(f.Solids), len(f.Faces))
for s in f.Solids: print(s.Volume, s.BoundBox)
for fc in f.Faces[:40]: print(fc.Surface.__class__.__name__, round(fc.Area,1))
