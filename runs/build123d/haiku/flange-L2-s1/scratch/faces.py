import pathlib
from build123d import *
p = pathlib.Path(__file__).parent.parent / "output.step"
s = import_step(str(p)).solids()[0]
rows=[]
for f in s.faces():
    n = f.normal_at(f.center())
    if n.Z < -0.2:
        bb=f.bounding_box()
        rows.append((round(n.Z,2), round(f.area,1), [round(x,1) for x in (bb.min.X,bb.min.Y,bb.min.Z)], [round(x,1) for x in (bb.max.X,bb.max.Y,bb.max.Z)], f.geom_type.name))
for r in sorted(rows, key=lambda r: r[0]): print(r)
print("total downward faces", len(rows))
