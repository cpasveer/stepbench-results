import sys, json, numpy as np
sys.path.insert(0, ".")
from build123d import *
import submission as S
p = S.build()
print("solids", len(p.solids()), "valid", p.is_valid, "vol", round(p.volume,1), "faces", len(p.faces()))
bb = p.bounding_box(); print(bb.min, bb.max)
A = import_step("fixed_A.step"); B = import_step("fixed_B.step")
for n, f in (("A", A), ("B", B)):
    print("overlap", n, (p & f).volume if (p & f) else 0)
RB = S.RB; up = -RB @ S.D
print("faces by type / overhang (normal.up at centre):")
for f in sorted(p.faces(), key=lambda f: -f.area):
    c = f.center(); nrm = f.normal_at(c)
    print(f"  {f.geom_type.name:9s} area {f.area:8.1f} n.up {np.dot(up, tuple(nrm)):6.3f}  cB {np.round(S.to_b(tuple(c)),1)}")
es = sorted(e.length for e in p.edges()); print("shortest edges", [round(x,3) for x in es[:6]])
