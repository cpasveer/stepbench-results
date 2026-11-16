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


def cyl(r, h, x, y, z):
    return Part.makeCylinder(r, h, V(x, y, z), V(0, 0, 1))


def arm(x0, y0, x1, y1, z, t, w, rpad):
    """Flat arm (width w) from (x0,y0) to (x1,y1) ending in a round pad."""
    import math
    L = math.hypot(x1 - x0, y1 - y0)
    ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
    b = Part.makeBox(L, w, t, V(0, -w / 2, 0))
    b.rotate(V(0, 0, 0), V(0, 0, 1), ang)
    b.translate(V(x0, y0, z))
    return b.fuse(cyl(rpad, t, x1, y1, z))


def build() -> Part.Shape:
    fa, fb = FACES[0], FACES[1]
    za, zb = fa["origin"][2], fb["origin"][2]
    mx = (fa["origin"][0] + fb["origin"][0]) / 2
    my = (fa["origin"][1] + fb["origin"][1]) / 2
    T = 5.0          # grip
    RC = 14.0        # column radius
    abolts = [b for b in BOLTS if b["axis"][2] > 0]
    bbolts = [b for b in BOLTS if b["axis"][2] < 0]
    s = Part.makeCylinder(RC, zb - za, V(mx, my, za), V(0, 0, 1))
    # A end (bottom): plate layer + 45 deg flare widening downward
    s = s.fuse(Part.makeCone(RC, 26.0, 12.0, V(mx, my, za + T + 12.0 - 12.0 + 0), V(0, 0, -1)).translate(V(0, 0, 0)) if False else
               Part.makeCone(26.0, RC, 12.0, V(mx, my, za + T), V(0, 0, 1)))
    s = s.fuse(cyl(26.0, T, mx, my, za))
    for b in abolts:
        p = b["position"]
        s = s.fuse(arm(mx, my, p[0], p[1], za, T, 10.0, 5.5))
    # B end (top): plate layer, flare widening upward, 50 deg from horizontal
    s = s.fuse(Part.makeCone(RC, 23.0, 10.0, V(mx, my, zb - T - 10.0), V(0, 0, 1)))
    s = s.fuse(cyl(23.0, T, mx, my, zb - T))
    for b in bbolts:
        p = b["position"]
        s = s.fuse(arm(mx, my, p[0], p[1], zb - T, T, 11.0, 6.0))
    for b in BOLTS:
        p = b["position"]
        d = 4.5 if b["size"] == "M4" else 5.5
        z0 = za - 1 if b["axis"][2] > 0 else zb - T - 1
        s = s.cut(cyl(d / 2, T + 2, p[0], p[1], z0))
    return s


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
