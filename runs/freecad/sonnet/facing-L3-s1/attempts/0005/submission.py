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
RT, WALL, WEB = 16.0, 2.4, 1.6
K = 1.35
ZA = FACES[0]["origin"][2]
ZB = FACES[1]["origin"][2]
TA, TB = 4.0, 4.7  # plate thickness (grip) at A and B
HWA = 4.8  # half-width of the A spokes
HWB = 4.6  # half-width (top) of the B spokes (flank 39 deg off vertical)
CUP_R, POCKET_R, POCKET_D = 6.0, 4.85, 6.4


def frame(bx, by):
    dx, dy = bx - PX, by - PY
    L = (dx * dx + dy * dy) ** 0.5
    u = V(dx / L, dy / L, 0)
    n = V(-u.y, u.x, 0)
    return L, u, n


def prism(L, u, n, z0, prof):
    """prof: list of (lateral, dz) closed polygon, extruded along u from the column to L."""
    s0 = RT - 1.8
    base = V(PX, PY, z0) + u * s0
    pts = [base + n * a + V(0, 0, dz) for a, dz in prof]
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(u * (L - s0))


def web(L, u, n, z0, sg, t, r0, top):
    s0 = RT - 1.0
    pts2 = [(s0, top), (L, top), (L, t), (L - r0, t), (s0, t + (L - r0 - s0) * K), (s0, top)]
    pts = [V(PX, PY, z0) + u * s + n * (-WEB / 2) + V(0, 0, sg * h) for s, h in pts2]
    return Part.Face(Part.makePolygon(pts)).extrude(n * WEB)


def build() -> Part.Shape:
    tubeH = ZB - ZA
    body = Part.makeCylinder(RT, tubeH, V(PX, PY, ZA), V(0, 0, 1))
    cuts = []
    for b in BOLTS:
        x, y, _ = b["position"]
        L, u, n = frame(x, y)
        if b["id"][0] == "A":
            hw = HWA
            body = body.fuse(prism(L, u, n, ZA, [(-hw, 0), (hw, 0), (hw, TA), (-hw, TA)]))
            body = body.fuse(Part.makeCylinder(hw, TA, V(x, y, ZA), V(0, 0, 1)))
            body = body.fuse(web(L, u, n, ZA, 1, TA, hw + 1.0, TA - 1.0))
            cuts.append(Part.makeCylinder(2.25, TA + 2, V(x, y, ZA - 1), V(0, 0, 1)))
        else:
            hw = HWB
            d = TB
            prof = [(-hw, 0), (hw, 0), (WEB / 2, -d), (-WEB / 2, -d)]
            body = body.fuse(prism(L, u, n, ZB, prof))
            body = body.fuse(Part.makeCylinder(CUP_R, TB + POCKET_D, V(x, y, ZB - TB - POCKET_D), V(0, 0, 1)))
            body = body.fuse(Part.makeCylinder(hw, 0.0 + TB, V(x, y, ZB - TB), V(0, 0, 1)))
            body = body.fuse(web(L, u, n, ZB, -1, d, CUP_R + 1.0, d - 1.0))
            cuts.append(Part.makeCylinder(2.75, TB + 2, V(x, y, ZB - TB - 1), V(0, 0, 1)))
            cuts.append(Part.makeCylinder(POCKET_R, POCKET_D + 1, V(x, y, ZB - TB - POCKET_D - 1), V(0, 0, 1)))
    body = body.removeSplitter()
    for h in cuts:
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
