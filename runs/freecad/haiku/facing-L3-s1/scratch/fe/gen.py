"""Generate candidate connector STEPs for local FE comparison. Usage: gen.py <name> <variant-json>"""
import json, math, sys, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
T = json.load(open(D + "task.json"))["poses"][0]["resolved"]
A = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "A"]
B = [tuple(b["position"][:2]) for b in T["bolts"] if b["id"][0] == "B"]
Ab = [b for b in T["bolts"] if b["id"][0] == "A"]
Bb = [b for b in T["bolts"] if b["id"][0] == "B"]
ca = (-126.0, 127.5); cb = (-124.18164228311065, 135.11206773573574)
import os
name = os.environ["GEN_NAME"]; cfg = json.loads(os.environ["GEN_CFG"])
P = tuple(cfg.get("P", cb))
ZA, ZB = -94.0, -25.5
def strip(p0, p1, w, z0, t):
    dx, dy = p1[0]-p0[0], p1[1]-p0[1]; L = math.hypot(dx, dy)
    nx, ny = -dy/L*w/2, dx/L*w/2
    pts = [V(p0[0]+nx, p0[1]+ny, z0), V(p1[0]+nx, p1[1]+ny, z0), V(p1[0]-nx, p1[1]-ny, z0), V(p0[0]-nx, p0[1]-ny, z0), V(p0[0]+nx, p0[1]+ny, z0)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, t))
def cyl(r, z0, z1, x, y):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))
parts = []
Rc = cfg.get("Rc", 8.0); Ztop = cfg.get("Ztop", ZB)
Rtop = cfg.get("Rtop", Rc)
if "Rtop" in cfg:
    parts.append(Part.makeCone(Rc, Rtop, Ztop - ZA, V(P[0], P[1], ZA), V(0, 0, 1)))
else:
    parts.append(cyl(Rc, ZA, Ztop, P[0], P[1]))
TA = cfg.get("TA", 4.0)
for q in A:
    parts.append(cyl(cfg.get("padA", 4.5), ZA, ZA + TA, q[0], q[1]))
    parts.append(strip(P, q, cfg.get("WA", 6.0), ZA, TA))
TB = cfg.get("TB", 5.0)
for q in B:
    parts.append(cyl(cfg.get("padB", 4.5), ZB - TB, ZB, q[0], q[1]))
    if cfg.get("style") == "strut":
        # strut from column top (inside) to pad tab, steep
        ux, uy = (P[0]-q[0]), (P[1]-q[1]); L = math.hypot(ux, uy); ux, uy = ux/L, uy/L
        tab_end = (q[0]+ux*6.0, q[1]+uy*6.0)
        parts.append(strip(q, tab_end, 6.0, ZB - TB, TB))
        s = V(q[0]+ux*5.5, q[1]+uy*5.5, ZB - TB/2)
        e = V(P[0]-ux*2.0, P[1]-uy*2.0, Ztop - 3.0)
        d = e.sub(s)
        parts.append(Part.makeCylinder(cfg.get("Rs", 2.4), d.Length, s, d.normalize()))
    else:
        # wedge gusset: flat top flush with B, flat underside out to 4.75 mm, then sloped down to the column
        gus = cfg.get("gus", 50.0)
        ux, uy = (P[0]-q[0]), (P[1]-q[1]); L = math.hypot(ux, uy); ux, uy = ux/L, uy/L
        sc = L - (Rtop - 0.5)                      # distance from bolt axis to column attach
        drop = (sc - 4.75) * math.tan(math.radians(gus))
        w = cfg.get("Wg", 6.0)
        nx, ny = -uy*w/2, ux*w/2
        pts2 = [(0.0, ZB), (sc, ZB), (sc, ZB - TB - drop), (4.75, ZB - TB), (0.0, ZB - TB)]
        def pt(sv, zv, sgn):
            return V(q[0] + ux*sv + sgn*nx, q[1] + uy*sv + sgn*ny, zv)
        face = Part.Face(Part.makePolygon([pt(sv, zv, -1) for sv, zv in pts2] + [pt(pts2[0][0], pts2[0][1], -1)]))
        gus_solid = face.extrude(V(2*nx, 2*ny, 0))
        parts.append(gus_solid)
    if cfg.get("style") == "strut":
        pass
bodyparts = parts
body = bodyparts[0]
for p in bodyparts[1:]:
    body = body.fuse(p)
body = body.removeSplitter()
out = D + "scratch/fe/" + name + ".step"
body.exportStep(out)
print("wrote", out, "volume", round(body.Volume, 1), "valid", body.isValid(), "solids", len(body.Solids))
