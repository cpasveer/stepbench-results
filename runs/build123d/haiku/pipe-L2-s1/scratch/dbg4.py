import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane
import submission as S
part = S.build()
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
b = S.BOLTS["B1"]; ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
cyl = Solid.make_cylinder(4.0, 5.0, Plane(pos+ax*8.0, perp(ax), ax))
inter = part & cyl
bb = inter.bounding_box()
print("overlap bbox min", bb.min, "max", bb.max, "vol", round(inter.volume,2))
print("B1 pos", pos, "ax", ax)
# height and lateral of the intersection
for f in inter.faces()[:6]:
    c = f.center(); print("face", round(f.area,2), (round(c.X,2),round(c.Y,2),round(c.Z,2)), f.geom_type.name)
