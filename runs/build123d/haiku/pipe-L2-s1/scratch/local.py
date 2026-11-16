import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane, import_step
import submission as S
part = S.build()
A = import_step(str(S.HERE/"fixed_A.step")); B = import_step(str(S.HERE/"fixed_B.step"))
def vol(x): return 0 if x is None else round(x.volume,3)
print("solids", len(part.solids()), "valid", part.is_valid, "vol", round(part.volume,1))
print("overlap A", vol(part & A), " overlap B", vol(part & B))
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
for bid,b in sorted(S.BOLTS.items()):
    ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
    M5 = b["size"]=="M5"; hh = 5.0 if M5 else 4.0; hd = 8.5 if M5 else 7.0
    grip = 8.0
    cyl = Solid.make_cylinder(hd/2+0.5, hh+1.0, Plane(pos+ax*grip, perp(ax), ax))
    sh = Solid.make_cylinder((hd/2+0.5)*0+ (4.75 if M5 else 4.0), 5.0, Plane(pos+ax*0.0, perp(ax), ax))
    print(bid, "head-space", vol(part & cyl))
# downward-facing faces that exceed 45 deg overhang (z-up printing)
bad=[]
for f in part.faces():
    n = f.normal_at(f.center())
    if n.Z < -0.707 and f.area > 0.5:
        bad.append((round(f.area,1), tuple(round(x,2) for x in (n.X,n.Y,n.Z)), tuple(round(x,1) for x in (f.center().X,f.center().Y,f.center().Z))))
print("overhang >45 faces:", bad)
print("min face area", round(min(f.area for f in part.faces()),3))
