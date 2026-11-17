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


def mat(rot, org):
    m = App.Matrix(rot[0][0], rot[0][1], rot[0][2], org[0],
                   rot[1][0], rot[1][1], rot[1][2], org[1],
                   rot[2][0], rot[2][1], rot[2][2], org[2], 0, 0, 0, 1)
    return m


def build() -> Part.Shape:
    PL = TASK["poses"][0]["placements"]
    mA = mat(PL["A"]["rotation"], PL["A"]["origin"])  # pipe frame -> world (x = axis)
    mB = mat(PL["B"]["rotation"], PL["B"]["origin"])  # B frame -> world (z = normal)

    # --- pipe saddle: shell r 34..42 hugging the pipe, +-50 deg about the bolt side
    R_IN, R_OUT, HALF_L, ANG = 34.0, 42.0, 26.0, 50.0
    ring = Part.makeCylinder(R_OUT, 2 * HALF_L, V(-HALF_L, 0, 0), V(1, 0, 0))
    ring = ring.cut(Part.makeCylinder(R_IN, 2 * HALF_L, V(-HALF_L, 0, 0), V(1, 0, 0)))
    import math
    t = math.tan(math.radians(ANG))
    wedge = Part.makePolygon([V(-HALF_L, 0, 0), V(-HALF_L, -60 * t, 60), V(-HALF_L, 60 * t, 60), V(-HALF_L, 0, 0)])
    wedge = Part.Face(wedge).extrude(V(2 * HALF_L, 0, 0))
    saddle = ring.common(wedge)

    # --- rib in the B frame (u, w plane, extruded along v)
    VW0, VW1 = -18.0, 6.0
    pts = [(-18, 0), (-18, 8), (-24, 70), (-52, 70), (-52, 60)]
    poly = Part.makePolygon([V(u, VW0, w) for u, w in pts] + [V(pts[0][0], VW0, pts[0][1])])
    rib = Part.Face(poly).extrude(V(0, VW1 - VW0, 0))
    plate = Part.makeBox(36, 36, 5, V(-18, -18, 0))

    # rib is built in B frame; bring pipe-frame shapes to B frame via world
    rib.transformShape(mB)
    plate.transformShape(mB)
    saddle.transformShape(mA)
    clipcyl = Part.makeCylinder(38.0, 400, V(-200, 0, 0), V(1, 0, 0))
    clipcyl.transformShape(mA)
    rib = rib.cut(clipcyl)
    body = saddle.fuse(rib).fuse(plate)

    # --- bolt holes
    for b in BOLTS:
        p = V(*b["position"]); a = V(*b["axis"])
        d = sum(b["clearance_diameter_range"]) / 2
        cyl = Part.makeCylinder(d / 2, 60, p - a * 10, a)
        body = body.cut(cyl)
        if b["id"].startswith("A"):  # flat spot face for the head on the round shell
            body = body.cut(Part.makeCylinder(6.0, 20, p + a * 7.0, a))
    return body


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
