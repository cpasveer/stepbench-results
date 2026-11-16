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


def build() -> Part.Shape:
    import math
    cA = V(*FACES[0]["origin"])
    cB = V(*FACES[1]["origin"])
    zA, zB = cA.z, cB.z
    t = 8.0
    # lower square plate, aligned to mount A (rotated -55 deg about vertical)
    plate = Part.makeBox(70, 70, t, V(-35, -35, 0))
    plate.rotate(V(0, 0, 0), V(0, 0, 1), -55)
    plate.translate(V(cA.x, cA.y, zA))
    # upper round plate under mount B
    disc = Part.makeCylinder(40.5, t, V(cB.x, cB.y, zB - t), V(0, 0, 1))
    mid = (cA + cB) * 0.5
    col = Part.makeCylinder(12, (zB - t) - (zA + t), V(mid.x, mid.y, zA + t), V(0, 0, 1))
    body = plate.fuse(col).fuse(disc)
    for b in BOLTS:
        p = b["position"]
        d = 4.5 if b["size"] == "M4" else 5.5
        z0 = zA - 1 if b["id"].startswith("A") else zB - t - 1
        body = body.cut(Part.makeCylinder(d / 2, t + 2, V(p[0], p[1], z0), V(0, 0, 1)))
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
