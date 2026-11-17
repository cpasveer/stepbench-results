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


def build() -> Part.Shape:
    import math
    pa = TASK["poses"][0]["placements"]["A"]
    R = pa["rotation"]
    org = V(*pa["origin"])
    m = App.Matrix(R[0][0], R[0][1], R[0][2], org.x,
                   R[1][0], R[1][1], R[1][2], org.y,
                   R[2][0], R[2][1], R[2][2], org.z,
                   0, 0, 0, 1)
    xa = V(R[0][0], R[1][0], R[2][0])
    za = V(R[0][2], R[1][2], R[2][2])
    RO, RI, HL = 37.0, 34.0, 27.0
    # saddle shell in local frame (pipe axis = local x)
    X = V(1, 0, 0)
    outer = Part.makeCylinder(RO, 2 * HL, V(-HL, 0, 0), X)
    inner = Part.makeCylinder(RI, 2 * HL + 2, V(-HL - 1, 0, 0), X)
    L = 60.0
    a50 = math.radians(50)
    wedge = Part.Face(Part.makePolygon([V(-HL - 1, 0, 0),
        V(-HL - 1, -L * math.sin(a50), L * math.cos(a50)),
        V(-HL - 1, L * math.sin(a50), L * math.cos(a50)),
        V(-HL - 1, 0, 0)])).extrude(V(2 * HL + 2, 0, 0))
    shell = outer.cut(inner).common(wedge)
    shell.transformShape(m)
    body = shell
    holes = []
    # bosses on pipe bolts
    for b in BOLTS:
        p = V(*b["position"]); ax = unit(b["axis"])
        if b["id"].startswith("A"):
            body = body.fuse(Part.makeCylinder(7.0, 7.0 - 1.5, p + ax * 1.5, ax))
        holes.append((p, ax, 5.5))
    # plate on B
    fb = FACES[0]
    o = V(*fb["origin"]); n = unit(fb["normal"])
    pb = TASK["poses"][0]["placements"]["B"]["rotation"]
    u = V(pb[0][1], pb[1][1], pb[2][1]); w = n.cross(u)
    u = w.cross(n)
    T = 10.0
    S = 17.0
    plate = Part.makeBox(2 * S, 2 * S, T, V(0, 0, 0))
    mm = App.Matrix(u.x, w.x, n.x, o.x, u.y, w.y, n.y, o.y, u.z, w.z, n.z, o.z, 0, 0, 0, 1)
    plate.transformShape(mm.multiply(App.Matrix(1,0,0,-S,0,1,0,-S,0,0,1,0,0,0,0,1)))
    body = body.fuse(plate)
    for b in BOLTS:
        if b["id"].startswith("B"):
            holes.append((V(*b["position"]), unit(b["axis"]), 4.5))
    # rod
    PA = org + za * 35.5
    PB = o + n * 5.0
    d = PB - PA
    ln = d.Length
    rod = Part.makeCylinder(7.0, ln, PA, d * (1.0 / ln))
    body = body.fuse(rod)
    for p, ax, dia in holes:
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
