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


def unit(v):
    v = V(*v)
    v.normalize()
    return v


def half_space(o, n, e1, e2):
    # big slab behind the B face (n < 0), used to clip the bar at the face
    mm = App.Matrix(e1.x, e2.x, n.x, o.x, e1.y, e2.y, n.y, o.y, e1.z, e2.z, n.z, o.z, 0, 0, 0, 1)
    sl = Part.makeBox(400, 400, 80, V(-200, -200, -80))
    sl.transformShape(mm)
    return sl


def build() -> Part.Shape:
    import math
    pa = TASK["poses"][0]["placements"]["A"]
    R = pa["rotation"]
    org = V(*pa["origin"])
    m = App.Matrix(R[0][0], R[0][1], R[0][2], org.x,
                   R[1][0], R[1][1], R[1][2], org.y,
                   R[2][0], R[2][1], R[2][2], org.z,
                   0, 0, 0, 1)
    za = V(R[0][2], R[1][2], R[2][2])
    RI, G, HL = 34.0, 6.0, 25.0
    D = RI + G                      # seat planes at this distance from pipe axis
    RV = D / math.cos(math.radians(17.5))   # polygon vertex radius
    # saddle: prism along the pipe axis, outer faces are flat seats square to the bolts
    pts = [V(-HL, 0, 0)]
    for ang in (-52.5, -17.5, 17.5, 52.5):
        a_ = math.radians(ang)
        pts.append(V(-HL, RV * math.sin(a_), RV * math.cos(a_)))
    pts.append(pts[0])
    fan = Part.Face(Part.makePolygon(pts)).extrude(V(2 * HL, 0, 0))
    pipe = Part.makeCylinder(RI, 2 * HL + 2, V(-HL - 1, 0, 0), V(1, 0, 0))
    shell = fan.cut(pipe)
    shell.transformShape(m)
    body = shell
    # plate on B: a shallow frustum (36 mm square on the face, 32.8 mm at the
    # bolt-head side) so no face leans past 45 degrees when printed standing
    fb = FACES[0]
    o = V(*fb["origin"]); n = unit(fb["normal"])
    pb = TASK["poses"][0]["placements"]["B"]["rotation"]
    e1 = V(pb[0][0], pb[1][0], pb[2][0]); e2 = V(pb[0][1], pb[1][1], pb[2][1])
    T = 6.0
    def sq(h, k):
        pp = [o + e1 * i * h + e2 * j * h + n * k for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        return Part.makePolygon(pp + [pp[0]])
    plate = Part.makeLoft([sq(18.0, 0.0), sq(16.4, T)], True, True)
    body = body.fuse(plate)
    # connecting bar (rhombic section, flat-sided at 38 deg from vertical so it
    # prints without support), horizontal when the pipe axis is vertical
    xa = V(R[0][0], R[1][0], R[2][0])
    xo = (o - org).dot(xa)
    PA = org + xa * xo + za * (D - 2.0)
    rel = PA - o
    e1A, e2A = rel.dot(e1), rel.dot(e2)
    e1h = -12.5 if e1A < 0 else 12.5
    r_ = e1h / e1A
    c = -e2A * r_ / (1 - r_)
    P0 = o + e2 * c
    d = PA - P0
    ln = d.Length
    r = d * (1.0 / ln)
    t = r.cross(xa); t.normalize()
    HV, HT = 8.5, 6.5
    Ps = P0 - r * 14.0
    pp = [Ps + xa * HV, Ps + t * HT, Ps - xa * HV, Ps - t * HT]
    bar = Part.Face(Part.makePolygon(pp + [pp[0]])).extrude(r * (ln + 14.0))
    bar = bar.cut(Part.makeBox(200, 200, 60, o - e1 * 100 - e2 * 100 - n * 60).transformGeometry(App.Matrix()) if False else
                  half_space(o, n, e1, e2))
    body = body.fuse(bar)
    for b in BOLTS:
        p = V(*b["position"]); ax = unit(b["axis"])
        dia = 5.5 if b["id"].startswith("A") else 4.5
        body = body.cut(Part.makeCylinder(dia / 2, 40, p - ax * 3, ax))
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
