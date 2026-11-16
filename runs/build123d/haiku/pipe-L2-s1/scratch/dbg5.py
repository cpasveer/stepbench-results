import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
P0 = Vector(*S.PIPE_PLACE["origin"])
Zp = Vector(*[S.PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
m = Vector(*S.BOLTS["A1"]["axis"]) + Vector(*S.BOLTS["A3"]["axis"])
X = (m - Zp*m.dot(Zp)).normalized()
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
t = Vector(0,0,1).cross(nb).normalized()
def bc(p):
    d = p - oB
    return dict(t=round(d.dot(t),2), z=round(d.Z,2), nb=round(d.dot(nb),2))
Pe = P0 + X*38.0 + Zp*S.WEB_PAD_S
Qe = oB + nb*(S.PLATE_T/2) + t*S.WEB_PLATE_DT + Vector(0,0,S.WEB_PLATE_Z - oB.Z)
print("Pe", bc(Pe), "Qe", bc(Qe))
print("B1 in B-frame", bc(Vector(*S.BOLTS['B1']['position'])))
