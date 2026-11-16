import sys, pathlib, math, itertools
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
P0 = Vector(*S.PIPE_PLACE["origin"])
Zp = Vector(*[S.PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
m = Vector(*S.BOLTS["A1"]["axis"]) + Vector(*S.BOLTS["A3"]["axis"])
X = (m - Zp*m.dot(Zp)).normalized(); Y = Zp.cross(X)
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
t = Vector(0,0,1).cross(nb).normalized()
bn = (nb*9.0 - Vector(0,0,1)*8.0).normalized()
rows=[]
for beta, dt, th, s_, r_ in itertools.product([3.0,5.0],[-4.0,0.0,4.0],[0,10,20,30,40],[-27,-24,-21,-18,-15],[36.0,38.0,40.0]):
    # root on the bevel plane at nb=beta, t offset dt
    z_b = oB.Z - 17 + beta*9.0/8.0
    Q = oB + nb*beta + t*dt + Vector(0,0,z_b - oB.Z)
    rad = X*math.cos(math.radians(th)) + Y*math.sin(math.radians(th))
    Pe = P0 + rad*r_ + Zp*s_
    D = (Pe-Q).normalized()
    ang_plate = math.degrees(math.asin(min(1,abs(D.dot(bn)))))
    ang_pad = math.degrees(math.asin(min(1,abs(D.dot(rad)))))
    ang_vert = math.degrees(math.acos(min(1,abs(D.Z))))
    if ang_vert<=40 and ang_pad>=40 and ang_plate>=45:
        rows.append((round(ang_pad,1), round(ang_vert,1), round(ang_plate,1), beta, dt, th, s_, r_, round((Pe-oB).dot(t),1)))
rows.sort(reverse=True)
for r_ in rows[:15]: print(r_)
print("n", len(rows))
