import sys, pathlib, math, itertools
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane, import_step
import submission as S
A = import_step(str(S.HERE/"fixed_A.step"))
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
def vol(x): return 0 if x is None else x.volume
def evaluate(part):
    out = {}
    out["valid"] = part.is_valid and len(part.solids())==1
    out["pipe"] = round(vol(part & A),2)
    hs = 0
    for bid,b in S.BOLTS.items():
        ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
        M5 = b["size"]=="M5"; hh = 5.0 if M5 else 4.0; hd = 8.5 if M5 else 7.0
        cyl = Solid.make_cylinder(hd/2+0.5, hh+1.0, Plane(pos+ax*8.0, perp(ax), ax))
        hs += vol(part & cyl)
    out["headspace"] = round(hs,2)
    bad=[]
    for f in part.faces():
        n = f.normal_at(f.center())
        if n.Z < -0.72 and f.area > 1.0: bad.append(round(f.area,1))
    out["overhang"] = bad
    return out
grid = []
for th in [-40, -30, 30, 40]:
  for s_ in [-27, -25, -23]:
    for r_ in [38.0, 40.0]:
      S.WEB_PAD_THETA = th; S.WEB_PAD_S = s_; S.WEB_PAD_R = r_
      try:
          part = S.build()
          res = evaluate(part)
      except Exception as e:
          res = {"error": str(e)[:80]}
      print(th, s_, r_, res, flush=True)
