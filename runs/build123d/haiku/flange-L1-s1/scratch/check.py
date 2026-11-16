import sys, math, json
sys.path.insert(0, ".")
from build123d import *
import submission as S
def V(x):
    return 0.0 if x is None else x.volume

body = S.build()
print("valid", body.is_valid, "solids", len(body.solids()), "vol", round(body.volume, 1))
fixedA = import_step("fixed_A.step"); fixedB = import_step("fixed_B.step")
print("overlap A (mm3)", round(V(body & fixedA), 4), " overlap B", round(V(body & fixedB), 4))

fixed = [fixedA, fixedB]
bolts = S.BOLTS
def sector(origin, u, s0, s1, r, th0, th1, n=40):
    pl = Plane(origin=origin + u * s0, z_dir=u)
    pts = [(0.0, 0.0)] + [(r * math.cos(math.radians(t)), r * math.sin(math.radians(t)))
                          for t in [th0 + (th1 - th0) * i / n for i in range(n + 1)]]
    # local 2D coords in plane pl
    w = [pl.to_world_coords(Vector(x, y, 0)) if hasattr(pl, "to_world_coords") else None for x, y in pts]
    return w, pl

for bid, b in bolts.items():
    u = Vector(*b["axis"]).normalized(); p = Vector(*b["position"])
    g = S.GRIP
    d = S.A_HOLE_D if b["size"] == "M5" else S.B_HOLE_D
    hr = (b["head_diameter"]) / 2
    # hole present on axis (should be outside body)
    hole_ok = not body.is_inside(p + u * 5.0)
    # seat ring coverage: ring from hole radius to head radius, on the seat plane, sampled just above
    cov = tot = 0
    for i in range(72):
        for rr in [d / 2 + 0.2 + k * (hr - d / 2 - 0.2) / 5 for k in range(6)]:
            ang = 2 * math.pi * i / 72
            ref = Vector(0, 0, 1)
            # build an orthonormal basis around u
            e1 = Vector(1, 0, 0).cross(u) if abs(u.X) < 0.9 else Vector(0, 1, 0).cross(u)
            e1 = e1.normalized(); e2 = u.cross(e1)
            q = p + (e1 * math.cos(ang) + e2 * math.sin(ang)) * rr + u * 0.05
            tot += 1
            cov += body.is_inside(q)
    # head clearance cylinder (radius head+0.5, from grip to head top +1)
    hc = Solid.make_cylinder(hr + 0.5, b["head_height"] + 1.0 + 0.02, Plane(origin=p + u * (g + 0.01), z_dir=u))
    head_vol = V(body & hc)
    print(f"{bid}: hole_axis_free={hole_ok} seat_ring={cov/tot:.2f} head_clear_overlap={head_vol:.4f}")

# key swing: long-leg sector at bend height above head top, 60 deg, radius 72, thickness 4
def key_free(bid, lean_bend=None):
    b = bolts[bid]
    u = Vector(*b["axis"]).normalized(); p = Vector(*b["position"])
    g = S.GRIP
    is_m5 = b["size"] == "M5"
    bend = g + b["head_height"] + (22.0 if is_m5 else 25.0)
    e1 = (Vector(1, 0, 0).cross(u) if abs(u.X) < 0.9 else Vector(0, 1, 0).cross(u)).normalized()
    e2 = u.cross(e1)
    obstacles = [body] + fixed
    for oid, ob in bolts.items():  # other heads
        if oid == bid: continue
        ou = Vector(*ob["axis"]).normalized(); op = Vector(*ob["position"])
        obstacles.append(Solid.make_cylinder(ob["head_diameter"] / 2 + 0.5, ob["head_height"] + 1, Plane(origin=op + ou * (S.GRIP), z_dir=ou)))
    R = 72.0
    free = []
    for th in range(0, 360, 5):
        t0 = math.radians(th); t1 = t0 + math.radians(60)
        pts = [Vector(0, 0, 0)] + [(e1 * math.cos(t) + e2 * math.sin(t)) * R for t in [t0 + (t1 - t0) * k / 24 for k in range(25)]]
        pts = [p + u * bend + v for v in pts]
        try:
            wire = Wire.make_polygon(pts + [pts[0]])
            face = Face(wire)
            sec = Solid.extrude(face, u * 4.0).moved(Location(-u * 2.0))
        except Exception as ex:
            print("sector err", ex); return None
        clash = any(V(sec & o) > 0.5 for o in obstacles)
        if not clash: free.append(th)
    return free

for bid in bolts:
    f = key_free(bid)
    print(bid, "free sweep starts (deg):", (f[:3] + ["..."] + f[-3:]) if f and len(f) > 6 else f, "count", len(f) if f else 0)
