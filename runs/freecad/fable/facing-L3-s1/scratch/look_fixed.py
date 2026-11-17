import sys
import Part
for n in "AB":
    s=Part.read(f"fixed_{n}.step")
    print(n,s.BoundBox,len(s.Solids),len(s.Faces),s.Volume)
    for f in s.Faces:
        if f.Area>200: print("  ",f.Surface.__class__.__name__,round(f.Area,1),f.BoundBox, len(f.Edges))
