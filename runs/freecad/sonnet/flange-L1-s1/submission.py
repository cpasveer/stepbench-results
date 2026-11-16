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
    return v.normalize()


def build() -> Part.Shape:
    T = 8.0
    fa, fb = FACES[0], FACES[1]
    oA, nA = V(*fa["origin"]), unit(V(*fa["normal"]))
    oB, nB = V(*fb["origin"]), unit(V(*fb["normal"]))
    # pad A: disc, pad B: rectangle (lateral = horizontal tangent, height = z)
    padA = Part.makeCylinder(34.0, T, oA, nA)
    tB = V(-nB.y, nB.x, 0)
    corner = oB - tB * 21 + V(0, 0, -46) - nB * 0
    padB = Part.makeBox(42.0, 92.0, T)
    # build box in local frame: x along tB, y along z, z along nB
    rot = App.Rotation(tB, V(0, 0, 1), nB, "XYZ")
    padB.Placement = App.Placement(corner, rot)
    # bridge bar
    tA = V(-nA.y, nA.x, 0)
    w = 10.0
    PA = [oA + nA * 4 + tA * w, oA + nA * 4 - tA * w]
    PB = [oB + nB * 4 + tB * w, oB + nB * 4 - tB * w]
    if (PA[0] - PB[0]).Length + (PA[1] - PB[1]).Length > (PA[0] - PB[1]).Length + (PA[1] - PB[0]).Length:
        PB.reverse()
    z0, z1 = -16.0, 2.0
    pts = [V(p.x, p.y, z0) for p in (PA[0], PA[1], PB[1], PB[0])]
    wire = Part.makePolygon(pts + [pts[0]])
    bar = Part.Face(wire).extrude(V(0, 0, z1 - z0))
    body = padA.fuse(padB).fuse(bar)
    for b in BOLTS:
        d = sum(b["clearance_diameter_range"]) / 2.0
        ax = V(*b["axis"])
        c = Part.makeCylinder(d / 2.0, T + 4, V(*b["position"]) - ax * 2, ax)
        body = body.cut(c)
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
