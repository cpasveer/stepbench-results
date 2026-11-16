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


import math

# ---- local frame: origin at A's face centre, x/y along A's square edges, z = world z
AO = V(*FACES[0]["origin"])
EX = V(0.5735765984126814, -0.8191521213976911, 0)
EY = V(0.8191521213976911, 0.5735765984126814, 0)
ZA = FACES[0]["origin"][2]          # -94.0, A mounting face (part sits above)
ZB = FACES[1]["origin"][2]          # -25.5, B mounting face (part sits below)


def to_local(p):
    d = V(*p) - AO
    return V(d.dot(EX), d.dot(EY), p[2])


T_A = 6.0      # A flange thickness (grip)
T_B = 6.0      # B flange thickness (grip)
BED = 34.5     # local y of the print-bed plane (part is printed on its side)
R_B = 41.25    # B seat radius: 84.5/2 - 1 mm setback
HALF_A = 35.5  # A seat half width: 73/2 - 1 mm setback
WALL = 6.0     # back web thickness (lies on the bed)
RIB = 6.0      # front rib thickness


def placed(shape):
    ang = math.degrees(math.atan2(EX.y, EX.x))
    pl = App.Placement(V(AO.x, AO.y, 0), App.Rotation(V(0, 0, 1), ang))
    out = shape.copy()
    out.transformShape(pl.toMatrix())
    return out


def build() -> Part.Shape:
    bc = to_local(FACES[1]["origin"])
    # A flange: square, corners away from the bed rounded
    a = Part.makeBox(2 * HALF_A, HALF_A + BED, T_A, V(-HALF_A, -HALF_A, ZA))
    edges = [e for e in a.Edges
             if abs(e.Vertexes[0].Point.x - e.Vertexes[1].Point.x) < 1e-6
             and abs(e.Vertexes[0].Point.y - e.Vertexes[1].Point.y) < 1e-6
             and e.Vertexes[0].Point.y < 0]
    a = a.makeFillet(8.0, edges)
    # B flange: round, trimmed flat at the bed plane
    b = Part.makeCylinder(R_B, T_B, V(bc.x, bc.y, ZB - T_B))
    b = b.common(Part.makeBox(200, 200, T_B + 2, V(bc.x - 100, BED - 200, ZB - T_B - 1)))
    # web: back wall on the bed + rib, spanning between the flanges
    h = (ZB - T_B) - (ZA + T_A)
    wall = Part.makeBox(40.0, WALL, h, V(-24.0, BED - WALL, ZA + T_A))
    rib = Part.makeBox(RIB, BED - WALL + 18.0, h, V(-RIB / 2, -18.0, ZA + T_A))
    body = a.fuse([b, wall, rib]).removeSplitter()
    # bolt holes
    for bolt in BOLTS:
        p = to_local(bolt["position"])
        d = 4.5 if bolt["size"] == "M4" else 5.5
        z0 = ZA - 1 if bolt["size"] == "M4" else ZB - T_B - 1
        t = T_A if bolt["size"] == "M4" else T_B
        body = body.cut(Part.makeCylinder(d / 2, t + 2, V(p.x, p.y, z0)))
    body = body.removeSplitter()
    return placed(body)


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
