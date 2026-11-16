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


def hull(pts):
    pts = sorted(set(pts))
    def cr(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo = []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    up = []
    for p in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def prism(poly_uz, C, n, t, o0, o1):
    pts = [C + t * u + n * o0 + V(0, 0, z - C.z) for (u, z) in poly_uz]
    w = Part.makePolygon(pts + [pts[0]])
    return Part.Face(w).extrude(n * (o1 - o0))


def build() -> Part.Shape:
    fa, fb = FACES
    CA = V(*fa["origin"]); nA = V(*fa["normal"]); tA = V(-nA.y, nA.x, 0)
    CB = V(*fb["origin"]); nB = V(*fb["normal"]); tB = V(-nB.y, nB.x, 0)
    TA, TB = 7.0, 8.0          # pad thicknesses (grip)
    ZS0, ZS1 = -73.0, -65.0    # bottom slab
    R = 35.5
    # pad A: disc plus a 60-degree teardrop pillar (no support needed)
    padA = Part.makeCylinder(R, TA, CA, nA)
    pil = [(30.7, -12.25), (15, -39.45), (15, ZS1 - 1), (-15, ZS1 - 1),
           (-15, -39.45), (-30.7, -12.25)]
    pilA = prism(pil, CA, nA, tA, 1.5, TA)
    padB = prism([(-21, -65), (21, -65), (21, 29), (-21, 29)], CB, nB, tB, 0, TB)
    # slab: hull of both footprints in plan
    plan = []
    for C, n, t, us, os_ in ((CA, nA, tA, (-15, 15), (1.5, TA)),
                             (CB, nB, tB, (-21, 21), (1.5, TB))):
        for u in us:
            for o in os_:
                p = C + t * u + n * o
                plan.append((round(p.x, 6), round(p.y, 6)))
    h = hull(plan)
    pts = [V(x, y, ZS0) for x, y in h]
    slab = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, 0, ZS1 - ZS0))
    body = padA.fuse(pilA).fuse(padB).fuse(slab)
    for bt in BOLTS:
        P = V(*bt["position"]); ax = V(*bt["axis"])
        d = 5.5 if bt["size"] == "M5" else 6.7
        body = body.cut(Part.makeCylinder(d / 2, 20, P - ax * 3, ax))
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
