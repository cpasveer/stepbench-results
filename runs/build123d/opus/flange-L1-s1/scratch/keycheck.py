"""Rough self-check: head clearance and hex-key swing for every bolt.
usage: python scratch/keycheck.py  (from task folder, after submission.py)"""
import json, math, pathlib, sys
from build123d import import_step, Solid, Plane, Vector, Compound

HERE = pathlib.Path(__file__).parent.parent
TASK = json.loads((HERE / "task.json").read_text())
BOLTS = TASK["poses"][0]["resolved"]["bolts"]
part = import_step(str(HERE / "output.step"))
fa = import_step(str(HERE / "fixed_A.step"))
fb = import_step(str(HERE / "fixed_B.step"))
obst = [part, fa, fb]
KEY = {"M5": (4.0, 25.0, 70.0, 3.0), "M6": (5.0, 28.0, 80.0, 3.5)}  # s, short, long, socket depth
T = {"A": 8.0, "B": 9.0}

def ortho(n):
    a = Vector(0, 0, 1) if abs(n.Z) < 0.9 else Vector(1, 0, 0)
    u = n.cross(a).normalized(); v = n.cross(u).normalized(); return u, v

for b in BOLTS:
    n = Vector(*b["axis"]).normalized(); p = Vector(*b["position"])
    t = T[b["id"][0]]
    s, Ls, Ll, sd = KEY[b["size"]]
    r = s / math.cos(math.radians(30)) / 2
    head_top = p + n * (t + b["head_height"])
    # head clearance volume
    hc = Solid.make_cylinder(b["head_diameter"] / 2 + 0.5, b["head_height"] + 1.0, Plane(origin=p + n * t, z_dir=n))
    x = hc & part; ov = x.volume if x is not None and x.solids() else 0.0
    u, v = ortho(n)
    tip = head_top - n * sd
    best = None
    for lean in (0, 12, 25):
        free = []
        for k in range(72):
            phi = math.radians(k * 5)
            d = u * math.cos(phi) + v * math.sin(phi)
            L = math.radians(lean)
            kdir = (n * math.cos(L) + d * math.sin(L)).normalized()
            ldir = (d * math.cos(L) - n * math.sin(L)).normalized()
            bend = tip + kdir * (Ls - r)
            short = Solid.make_cylinder(r, Ls - r - sd - 0.3, Plane(origin=tip + kdir * (sd + 0.3), z_dir=kdir))
            lng = Solid.make_cylinder(r, Ll, Plane(origin=bend - ldir * r, z_dir=ldir))
            key = short + lng
            dist = min(key.distance_to(o) for o in obst)
            free.append(dist)
        # best 60-degree window (13 samples)
        for k0 in range(72):
            w = min(free[(k0 + j) % 72] for j in range(13))
            if best is None or w > best[0]:
                best = (w, lean, k0 * 5)
    print(f"{b['id']}: head overlap {ov:.3f}  best key window clearance {best[0]:.2f} mm (lean {best[1]}, start {best[2]} deg)")
    sys.stdout.flush()
