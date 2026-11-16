import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Plane, Solid
import submission as S
P0 = Vector(*S.PIPE_PLACE["origin"])
Zp = Vector(*[S.PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
m = Vector(*S.BOLTS["A1"]["axis"]) + Vector(*S.BOLTS["A3"]["axis"])
X = (m - Zp*m.dot(Zp)).normalized(); Y = Zp.cross(X)
n = (X - Zp).normalized()
o = P0 + X*S.PIPE_R + Zp*S.PAD_S_BOT
hs = S.half_space(o, n, Y, None)
for r in [34.5, 36.9, 40]:
    p = P0 + X*r + Zp*(-31.0)
    print("r", r, "in half-space:", hs.is_inside(p), " dot", round((p-o).dot(n),3))
print("hs bbox", hs.bounding_box().min, hs.bounding_box().max)
