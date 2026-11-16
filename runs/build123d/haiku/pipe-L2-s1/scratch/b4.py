import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane
import submission as S
part = S.build()
b = S.BOLTS["B4"]; ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
for h0, h1 in [(-2, 0.5), (0.5, 8), (8, 13), (13, 20)]:
    cyl = Solid.make_cylinder(4.75, h1-h0, Plane(pos + ax*h0, perp(ax), ax))
    inter = part & cyl
    if inter is None: print(h0,h1,"none"); continue
    bb = inter.bounding_box()
    print("h", h0, h1, "vol", round(inter.volume,3), "bbox", tuple(round(x,2) for x in (bb.min.X,bb.min.Y,bb.min.Z)), tuple(round(x,2) for x in (bb.max.X,bb.max.Y,bb.max.Z)))
