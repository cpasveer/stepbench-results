import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import *
import submission as S
part = import_step(str(S.HERE/"output.step"))
def loc(p):
    r = p - S.OA; rb = p - S.OB
    return "A(d=%.2f u=%.2f) B(x=%.2f y=%.2f) z=%.2f" % (r.dot(S.NA), r.dot(S.TA_DIR), rb.dot(S.XB), rb.dot(S.NB), p.Z)
for f in part.faces():
    if f.area < 13: print("face", round(f.area,3), f.geom_type, loc(f.center()))
for e in part.edges():
    if e.length < 1.1: print("edge", round(e.length,3), loc(e.center()))
