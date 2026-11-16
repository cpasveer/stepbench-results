import sys, pathlib, math, random
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
part = S.build()
faces = []
for f in part.faces():
    n = f.normal_at(f.center())
    faces.append((f.area, Vector(n.X,n.Y,n.Z)))
def worst(d):
    w = 0.0; area = 0.0
    for a, n in faces:
        s = n.dot(d)   # >0 means face points up
        if -0.999 < s < -1e-6 and a > 1.0:
            ov = math.degrees(math.asin(min(1.0,-s)))
            w = max(w, ov); area += a
    return w, area
random.seed(1)
best = []
N = 6000
for i in range(N):
    z = random.uniform(-1,1); phi = random.uniform(0, 2*math.pi)
    r = math.sqrt(1-z*z); d = Vector(r*math.cos(phi), r*math.sin(phi), z)
    w, a = worst(d)
    best.append((round(w,1), round(a,1), tuple(round(x,3) for x in (d.X,d.Y,d.Z))))
best.sort()
for b in best[:8]: print(b)
print("worst for world Z up:", worst(Vector(0,0,1)))
print("--- bed = each large planar face ---")
from build123d import Vector as V
for f in part.faces():
    if f.geom_type.name == "PLANE" and f.area > 100:
        n = f.normal_at(f.center())
        d = V(-n.X, -n.Y, -n.Z)  # face lies on the bed: its outward normal points down
        w, a = worst(d)
        print("area", round(f.area,1), "bed normal", tuple(round(x,2) for x in (n.X,n.Y,n.Z)), "worst", round(w,1), "down area", round(a,1))
