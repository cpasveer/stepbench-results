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
zb = oB.Z - S.PLATE_HALF
Qe = oB + nb*S.WEB_PLATE_BETA + t*S.WEB_PLATE_DT + Vector(0,0,abs(S.WEB_PLATE_BETA-S.PLATE_T/2)*S.PLATE_BEVEL[1]/S.PLATE_BEVEL[0] + zb - oB.Z)
th = math.radians(S.WEB_PAD_THETA)
rad = X*math.cos(th) + Y*math.sin(th)
Pe = P0 + rad*S.WEB_PAD_R + Zp*S.WEB_PAD_S
D = (Pe-Qe).normalized()
slope = (nb*S.PLATE_BEVEL[1] - Vector(0,0,1)*S.PLATE_BEVEL[0]).normalized()
print("joint plate-slope", round(math.degrees(math.asin(abs(D.dot(slope)))),1))
print("joint pad", round(math.degrees(math.asin(abs(D.dot(rad)))),1))
print("from vertical", round(math.degrees(math.acos(abs(D.Z))),1))
