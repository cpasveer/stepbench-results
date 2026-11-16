import Part
s=Part.read("output.step")
for n in ("fixed_A.step","fixed_B.step"):
    f=Part.read(n); print(n, "overlap", s.common(f).Volume)
for i,f in enumerate(s.Faces): print(i, f.Surface.__class__.__name__, round(f.Area,2), len(f.Edges))
print(min(e.Length for e in s.Edges))
