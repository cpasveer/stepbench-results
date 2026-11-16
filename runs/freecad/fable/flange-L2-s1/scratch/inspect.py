import Part, FreeCAD as App
s = Part.read("output.step")
print("valid", s.isValid(), "solids", len(s.Solids), "vol", s.Volume, "bbox", s.BoundBox)
for n in ("fixed_A.step", "fixed_B.step"):
    f = Part.read(n)
    print(n, f.BoundBox, "common", s.common(f).Volume)
for i, f in enumerate(s.Faces):
    print(i, f.Surface.__class__.__name__, "A=%.2f" % f.Area, "edges", len(f.Edges))
print("min edge", min(e.Length for e in s.Edges), "nedges", len(s.Edges))
