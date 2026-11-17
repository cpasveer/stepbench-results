"""Your answer to this task, as a FreeCAD script.

Write ONE solid to output.step (next to this file) that bolts to both fixed
parts. Read README.md for the brief; every number you need is in task.json.

Only output.step is graded. You may replace this script entirely, or model
the part in FreeCAD any other way and save it as output.step instead.

    freecadcmd submission.py             # writes output.step
    jointbench verify . --by <your name> --tool freecad

(the brief, README.md, says how to run them.) Run both from this folder.

FreeCAD's Part API, all in world coordinates (millimetres):
Part.makeBox(l, w, h, App.Vector(base), App.Vector(dir)),
Part.makeCylinder(r, h, App.Vector(base), App.Vector(dir)), a.fuse(b),
a.cut(b), shape.removeSplitter(), shape.transformShape(App.Matrix),
App.Placement / App.Rotation / App.Vector. The fixed parts load with
Part.read("fixed_A.step"). No document or GUI is needed.
"""
import json
import os
import sys
import traceback

import FreeCAD as App
import Part

# freecadcmd does not always set __file__; the task folder is this file's
# folder, else the folder you run it from.
try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]  # [{"part", "origin", "normal"}]
BOLTS = POSE["bolts"]  # [{"id", "size", "position", "axis", "clearance_diameter_range", ...}]
V = App.Vector


def unit(v):
    v = V(*v)
    v.normalize()
    return v


def basis_matrix(o, a, b, c):
    return App.Matrix(a.x, b.x, c.x, o.x, a.y, b.y, c.y, o.y, a.z, b.z, c.z, o.z, 0, 0, 0, 1)


def build() -> Part.Shape:
    import math
    ALPHA = math.radians(float(os.environ.get("ALPHA", "-30")))
    ELEV = math.radians(float(os.environ.get("ELEV", "-30")))
    pa = TASK["poses"][0]["placements"]["A"]
    R = pa["rotation"]
    org = V(*pa["origin"])
    xa = V(R[0][0], R[1][0], R[2][0])
    ya = V(R[0][1], R[1][1], R[2][1])
    za = V(R[0][2], R[1][2], R[2][2])
    RI, G = 34.0, 10.0
    D = RI + G                       # seat planes lie this far from the pipe axis
    RV = D / math.cos(math.radians(17.5))
    # --- B side frame
    fb = FACES[0]
    o = V(*fb["origin"]); n = unit(fb["normal"])
    pb = TASK["poses"][0]["placements"]["B"]["rotation"]
    e1 = V(pb[0][0], pb[1][0], pb[2][0]); e2 = V(pb[0][1], pb[1][1], pb[2][1])
    # --- print direction (up when printing): the pipe axis tipped toward -e2
    u = xa * math.cos(ALPHA) - e2 * math.sin(ALPHA)
    u.normalize()
    # --- saddle: prism along the pipe axis, flat seat faces square to the bolts
    pts = [V(-90, 0, 0)]
    for ang in (-52.5, -17.5, 17.5, 52.5):
        a_ = math.radians(ang)
        pts.append(V(-90, RV * math.sin(a_), RV * math.cos(a_)))
    pts.append(pts[0])
    fan = Part.Face(Part.makePolygon(pts)).extrude(V(180, 0, 0))
    pipe = Part.makeCylinder(RI, 182, V(-91, 0, 0), V(1, 0, 0))
    shell = fan.cut(pipe)
    shell.transformShape(basis_matrix(org, xa, ya, za))
    # --- plate frustum on B
    T = 6.0
    def sq(h, k):
        pp = [o + e1 * i * h + e2 * j * h + n * k for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        return Part.makePolygon(pp + [pp[0]])
    plate = Part.makeLoft([sq(18.0, 0.0), sq(16.4, T)], True, True)
    # --- bar: rhombic section, climbing ELEV above the horizontal toward the saddle
    def geom(xs):
        tgt = org + xa * xs + za * (D - 2.0)
        rel = tgt - o
        e1A, e2A = rel.dot(e1), rel.dot(e2)
        e1h = -12.5 if e1A < 0 else 12.5
        r_ = e1h / e1A
        c = -e2A * r_ / (1 - r_)
        P0 = o + e2 * c
        dd = tgt - P0
        L = dd.Length
        return tgt, P0, dd * (1.0 / L), L
    lo, hi = -80.0, 120.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        el = geom(mid)[2].dot(u)
        if el < math.sin(ELEV):
            lo = mid
        else:
            hi = mid
    PEnd, P0, r, k = geom(0.5 * (lo + hi))
    t = r.cross(u); t.normalize()
    v = r.cross(t); v.normalize()
    HV, HT = 8.5, 6.5
    Ps = P0 - r * 14.0
    pp = [Ps + v * HV, Ps + t * HT, Ps - v * HV, Ps - t * HT]
    bar = Part.Face(Part.makePolygon(pp + [pp[0]])).extrude(r * (k + 14.0))
    sl = Part.makeBox(400, 400, 80, V(-200, -200, -80))
    sl.transformShape(basis_matrix(o, e1, e2, n))
    bar = bar.cut(sl)
    # keep the bar's footprint on the B face well inside the plate outline
    lim = Part.makeBox(32.0, 32.0, 80, V(-16.0, -16.0, 0))
    lim.transformShape(basis_matrix(o, e1, e2, n))
    ring = Part.makeBox(300, 300, T, V(-150, -150, 0))
    ring.transformShape(basis_matrix(o, e1, e2, n))
    bar = bar.cut(ring.cut(lim))
    # --- bed and top planes of the saddle, square to the print direction
    hb = [((V(*b["position"]) + unit(b["axis"]) * G - org).dot(u)) for b in BOLTS if b["id"].startswith("A")] + [((V(*b["position"]) - org).dot(u)) for b in BOLTS if b["id"].startswith("A")]
    hmin = min(hb) - 6.0
    hmax = max(max(hb) + 6.0, (PEnd - org).dot(u) + HV + 3.0)
    uu = u.cross(V(1, 0, 0) if abs(u.x) < 0.9 else V(0, 1, 0)); uu.normalize()
    vv = u.cross(uu)
    cut = Part.makeBox(500, 500, 500, V(-250, -250, 0))
    cut.transformShape(basis_matrix(org, uu, vv, u))
    keep = cut.copy(); keep.translate(u * hmin)
    top = cut.copy(); top.translate(u * hmax)
    shell = shell.common(keep.cut(top))
    body = shell.fuse(plate).fuse(bar)
    for b in BOLTS:
        p = V(*b["position"]); ax = unit(b["axis"])
        dia = 5.5 if b["id"].startswith("A") else 4.5
        body = body.cut(Part.makeCylinder(dia / 2, 40, p - ax * 3, ax))
    print("bar end x", (PEnd - org).dot(xa), "y", (PEnd - org).dot(ya), "len", k)
    return body.removeSplitter()


# freecadcmd exits 0 even when a script fails; this makes a failure visible
# (full traceback, exit 1) and never leaves a stale output.step behind.
OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
