import sys, json, pathlib, math
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import *
import submission as S
part = import_step(str(S.HERE/"output.step"))
print("solids", len(part.solids()), "faces", len(part.faces()), "vol", part.volume)
bb = part.bounding_box(); print("bbox", bb.min, bb.max)
for n in "AB":
    f = import_step(str(S.HERE/f"fixed_{n}.step"))
    fb = f.bounding_box()
    print(n, "bbox", fb.min, fb.max, "overlap", (part & f).volume if (part & f) else 0)
for b in S.BOLTS:
    ax = Vector(*b["axis"]); p = Vector(*b["position"])
    grip = S.T_A if b["size"]=="M5" else S.T_B
    hd, hh = b["head_diameter"], b["head_height"]
    def cyl(r, a, l):
        pl = Plane(origin=p+ax*a, z_dir=ax)
        return Solid.extrude(Face(Wire.make_circle(r, pl)), ax*l)
    head = cyl(hd/2+0.5, grip+0.01, hh+1)
    tool = cyl(hd/2, grip+hh, 30)
    vh = part & head; vt = part & tool
    print(b["id"], "head", round(vh.volume,3) if vh else 0, "tool", round(vt.volume,3) if vt else 0)
# short edges / small faces
es = sorted(e.length for e in part.edges()); print("min edges", [round(x,3) for x in es[:6]])
fs = sorted(f.area for f in part.faces()); print("min faces", [round(x,3) for x in fs[:6]])
