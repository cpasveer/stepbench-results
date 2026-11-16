"""Own pre-checks: overhangs, interference, bolt envelopes, face stats."""
import sys; sys.path.insert(0,'.')
from submission import *
from build123d import import_step, GeomType
def vol(x):
    return 0.0 if x is None else sum(s.volume for s in x.solids())
part = build().solids()[0]
print("volume", part.volume, "valid", part.is_valid, "faces", len(part.faces()))
fa = import_step("fixed_A.step"); fb = import_step("fixed_B.step")
print("B bbox", fb.bounding_box())
for nm, f in (("A", fa), ("B", fb)):
    print("overlap", nm, vol(part & f))
# faces
print("-- faces: type area  normal.up(at centre)  height")
for f in sorted(part.faces(), key=lambda f: -f.area):
    c = f.center(); 
    try: n = f.normal_at(c)
    except Exception: n = None
    nu = float(np.array([n.X,n.Y,n.Z])@UP) if n else None
    h = float((np.array([c.X,c.Y,c.Z])-E0)@UP)
    print(f"{str(f.geom_type):22s} {f.area:9.2f} n.up={nu:+.3f} h={h:7.2f}")
print("min edge", min(e.length for e in part.edges()))
# bolt envelopes
HEAD = {"M5": (8.5, 5.0), "M4": (7.0, 4.0)}
for k, b in BOLTS.items():
    p, ax = np.array(b["position"]), np.array(b["axis"])
    x = unit(np.cross(ax, [0.3, 0.5, 0.8]))
    grip = RAIL_GRIP if b["size"] == "M5" else PLATE_T
    hd, hh = HEAD[b["size"]]
    L = hh + 1 + 30
    head = plane(p + (grip + 0.01 + (hh+1) / 2) * ax, x, ax) * Cylinder(hd / 2 + 0.5, hh+1)
    tool = plane(p + (grip + hh + 15) * ax, x, ax) * Cylinder(hd / 2, 30)
    v1 = vol(part & head); v2 = vol(part & tool)
    v3 = vol(fa & tool)+vol(fb & tool)
    print(k, "head clash", round(v1, 3), "tool clash", round(v2, 3), "fixed", round(v3,3))
