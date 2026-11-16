import sys, pathlib, math
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
P0 = Vector(*S.PIPE_PLACE["origin"])
Zp = Vector(*[S.PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
m = Vector(*S.BOLTS["A1"]["axis"]) + Vector(*S.BOLTS["A3"]["axis"])
X = (m - Zp*m.dot(Zp)).normalized(); Y = Zp.cross(X)
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
Qe = oB + nb*4.0 + Vector(0,0,S.WEB_PLATE_Z - oB.Z)
th = math.radians(S.WEB_PAD_THETA)
rad = X*math.cos(th) + Y*math.sin(th)
Pe = P0 + rad*S.WEB_PAD_R + Zp*S.WEB_PAD_S
D = (Pe - Qe).normalized()
print("web dir", D)
print("angle web to plate outer face (deg):", round(math.degrees(math.asin(abs(D.dot(nb)))),1))
print("angle web to pad surface (deg):", round(math.degrees(math.asin(abs(D.dot(rad)))),1))
print("angle from vertical (deg):", round(math.degrees(math.acos(abs(D.Z))),1))
