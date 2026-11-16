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


PX, PY = -125.1, 131.3
RT, WALL, WEB = 12.0, 1.4, 1.6
K = 1.35
ZA = FACES[0]["origin"][2]
ZB = FACES[1]["origin"][2]


def web(bx, by, z0, sg, t, S, rboss):
    """Gusset from the column to a bolt boss; sg=+1 grows up from z0, -1 down."""
    dx, dy = bx - PX, by - PY
    L = (dx * dx + dy * dy) ** 0.5
    u = V(dx / L, dy / L, 0)
    n = V(-u.y, u.x, 0)
    s0 = RT - 1.0
    r0 = rboss + 1.0
    pts2 = [(s0, 0), (L, 0), (L, t), (L - r0, t), (s0, t + (L - r0 - s0) * K), (s0, 0)]
    pts = [V(PX, PY, z0) + u * s + n * (-WEB / 2) + V(0, 0, sg * h) for s, h in pts2]
    w = Part.Face(Part.makePolygon(pts))
    return w.extrude(n * WEB)


def build() -> Part.Shape:
    tubeH = ZB - ZA
    body = Part.makeCylinder(RT, tubeH, V(PX, PY, ZA), V(0, 0, 1))
    holes = []
    for b in BOLTS:
        x, y, _ = b["position"]
        isA = b["id"][0] == "A"
        rb = 4.8 if isA else 5.3
        t = 4.0
        z0 = ZA if isA else ZB
        sg = 1 if isA else -1
        zb = min(z0, z0 + sg * t)
        body = body.fuse(Part.makeCylinder(rb, t, V(x, y, zb), V(0, 0, 1)))
        body = body.fuse(web(x, y, z0, sg, t, 0, rb))
        d = 4.5 if isA else 5.5
        holes.append(Part.makeCylinder(d / 2, t + 2, V(x, y, zb - 1), V(0, 0, 1)))
    body = body.removeSplitter()
    for h in holes:
        body = body.cut(h)
    bore = Part.makeCylinder(RT - WALL, tubeH + 2, V(PX, PY, ZA - 1), V(0, 0, 1))
    body = body.cut(bore)
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
