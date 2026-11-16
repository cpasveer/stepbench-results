import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane
import submission as S
part = S.build()
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
for bid in ["B1","B2","B3","B4"]:
    b = S.BOLTS[bid]; ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
    out=[]
    for r in [2.0, 2.25, 2.3, 2.5]:
        cyl = Solid.make_cylinder(r, 9.0, Plane(pos + ax*(-1.0), perp(ax), ax))
        i = part & cyl
        out.append((r, 0 if i is None else round(i.volume,3)))
    print(bid, out)
