import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
part = S.build()
rows=[]
for f in part.faces():
    n = f.normal_at(f.center()); c = f.center()
    if n.Z < -0.5 and f.area > 0.3:
        rows.append((round(f.area,2), round(n.Z,3), f.geom_type.name, tuple(round(x,1) for x in (c.X,c.Y,c.Z))))
for r in sorted(rows, reverse=True): print(r)
print("n faces", len(part.faces()))
