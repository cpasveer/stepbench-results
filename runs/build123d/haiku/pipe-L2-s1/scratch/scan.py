import sys, pathlib, math
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
P0 = Vector(*S.PIPE_PLACE["origin"])
Zp = Vector(*[S.PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
m = Vector(*S.BOLTS["A1"]["axis"]) + Vector(*S.BOLTS["A3"]["axis"])
X = (m - Zp*m.dot(Zp)).normalized(); Y = Zp.cross(X)
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
t = Vector(0,0,1).cross(nb).normalized()
Qe = oB + nb*4.0 + Vector(0,0,56.0-oB.Z)
res=[]
for th in range(-50, 51, 5):
  for s in [-26, -23, -20, -15]:
    r = math.radians(th)
    rad = X*math.cos(r) + Y*math.sin(r)
    Pe = P0 + rad*38.0 + Zp*s
    D = Pe - Qe
    ang = math.degrees(math.acos(abs(D.Z)/D.length))
    res.append((round(ang,1), th, s, round((Pe-oB).dot(t),1), round((Pe-oB).Z,1), round((Pe-oB).dot(nb),1)))
# want |t| small, angle small
res = [r for r in res if abs(r[3]) < 4]
res.sort()
for r_ in res[:10]: print(r_)
print("count", len(res))
