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
half_arc = 60
def rot(vec, deg):
    import math
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return vec*c + Zp.cross(vec)*s
length = S.PAD_S_TOP - S.PAD_S_BOT
sp = Plane(P0 + Zp*S.PAD_S_BOT, rot(X, -50), Zp)
outer = Solid.make_cylinder(S.PAD_OUT_R, length, sp, 100)
inner = Solid.make_cylinder(S.PIPE_R, length+2, Plane(P0 + Zp*(S.PAD_S_BOT-1), X, Zp))
pad = outer - inner
print("pad vol", round(pad.volume,1))
cut = pad - S.half_space(o, n, Y, None)
print("cut vol", round(cut.volume,1))
for f in cut.faces():
    nn = f.normal_at(f.center())
    if abs(nn.dot(Zp)) > 0.9:
        print("axial face", round(f.area,1), round(nn.dot(Zp),2), "s=", round((f.center()-P0).dot(Zp),2))
