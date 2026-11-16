import json, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import import_step, Vector
import submission as S

part = S.build()
A = import_step(str(S.HERE / "fixed_A.step"))
B = import_step(str(S.HERE / "fixed_B.step"))
print("overlap A mm3:", (lambda s: 0 if s is None else round(s.volume,4))(part & A))
print("overlap B mm3:", (lambda s: 0 if s is None else round(s.volume,4))(part & B))
bb = part.bounding_box()
print("bbox", bb.min, bb.max)
# min height above B face (along nb) and below
oB = Vector(*S.FACE["origin"]); nb = Vector(*S.FACE["normal"]).normalized()
hs = [ (vv.X - oB.X)*nb.X + (vv.Y-oB.Y)*nb.Y + (vv.Z-oB.Z)*nb.Z for vv in [Vector(q.X,q.Y,q.Z) for q in part.vertices()]]
print("min/max height along B normal:", round(min(hs),3), round(max(hs),3))
# radial distance of pad vertices from pipe axis
P0 = Vector(*S.PIPE_PLACE["origin"]); Zp = Vector(*[S.PIPE_PLACE["rotation"][r][0] for r in range(3)]).normalized()
rs = []
for vv0 in part.vertices():
    vv = Vector(vv0.X, vv0.Y, vv0.Z)
    d = vv - P0; ax = d.dot(Zp); rad = (d - Zp*ax).length
    rs.append(rad)
print("min radial distance from pipe axis:", round(min(rs),3))
