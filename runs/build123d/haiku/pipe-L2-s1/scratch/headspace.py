import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane
import submission as S
part = S.build()
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
for bid, b in sorted(S.BOLTS.items()):
    ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
    M5 = b["size"]=="M5"
    grip = 8.0; hh = 5.0 if M5 else 4.0; hd = 8.5 if M5 else 7.0
    r = hd/2 + 0.5
    # head space: from grip height (above surface) up to head top + 1 mm
    start = pos + ax*grip
    cyl = Solid.make_cylinder(r, hh + 1.0, Plane(start, perp(ax), ax))
    inter = part & cyl
    vol = 0 if inter is None else round(inter.volume, 3)
    # key band: cylinder 70/63 mm, 3 mm above head top, check count of volume
    print(bid, "head-space overlap mm3:", vol)
