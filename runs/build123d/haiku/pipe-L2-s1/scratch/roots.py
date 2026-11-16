import sys, pathlib, math
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
P0 = Vector(*S.PIPE_PLACE["origin"])
cols = [[S.PIPE_PLACE["rotation"][r][c] for r in range(3)] for c in range(3)]
Zp = Vector(*cols[0]).normalized()
m = Vector(*S.BOLTS["A1"]["axis"]) + Vector(*S.BOLTS["A3"]["axis"])
X = (m - Zp*m.dot(Zp)).normalized(); Y = Zp.cross(X)
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
Zw = Vector(0,0,1)
best=[]
for s in [-30,-27,-24,-20,-10,0,10,20]:
  for th in [-30,-15,0,15,30]:
    r=38.0
    th_r=math.radians(th)
    rad = X*math.cos(th_r)+Y*math.sin(th_r)
    P = P0 + rad*r + Zp*s
    for tq in [-6,0,6]:
      for zq in [40,46,52]:
        Q = oB + nb*5 + Vector(*(0,0,0)) + S_t if False else None
    # plate root candidates: point at nb=5, t,z offsets
    t = (Zw.cross(nb)).normalized()
    for dt in [-6,0,6]:
      for zq in [44,50,56]:
        Q = oB + nb*5 + t*dt + Zw*(zq-oB.Z)
        D = P-Q
        ang = math.degrees(math.acos(abs(D.Z)/D.length))  # angle from vertical
        best.append((round(ang,1), s, th, dt, zq, round(D.length,1), (round(P.X,1),round(P.Y,1),round(P.Z,1))))
best.sort()
for b in best[:12]: print(b)
