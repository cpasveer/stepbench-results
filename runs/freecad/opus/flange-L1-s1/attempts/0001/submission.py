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


T_A = 8.0          # flange plate thickness on A (bolt grip)
T_B = 8.0          # plate thickness on B (bolt grip)
R_A = 36.25 - 0.5  # A plate radius: 1.5 mm inside the 74.5 mm face
B_W, B_H = 46.0 - 3.0, 96.0 - 3.0  # B plate, 1.5 mm inside the face outline
R_HUB_A = 12.0     # neck radius where it meets the A plate
R_HUB_B = 15.0     # neck radius where it meets the B plate


def frame(name):
    p = TASK["poses"][0]["placements"][name]
    R = p["rotation"]
    o = V(*p["origin"])
    x = V(R[0][0], R[1][0], R[2][0])
    y = V(R[0][1], R[1][1], R[2][1])
    z = V(R[0][2], R[1][2], R[2][2])
    return o, x, y, z


def placed(shape, o, x, y, z):
    m = App.Matrix(x.x, y.x, z.x, o.x, x.y, y.y, z.y, o.y, x.z, y.z, z.z, o.z, 0, 0, 0, 1)
    s = shape.copy()
    s.transformShape(m)
    return s


def build() -> Part.Shape:
    oA, xA, yA, zA = frame("A")
    oB, xB, yB, zB = frame("B")
    # A: round flange plate
    plateA = Part.makeCylinder(R_A, T_A)
    plateA = plateA.makeFillet(1.5, [e for e in plateA.Edges if abs(e.BoundBox.ZMin - T_A) < 1e-6])
    plateA = placed(plateA, oA, xA, yA, zA)
    # B: rectangular plate with rounded corners
    plateB = Part.makeBox(B_W, B_H, T_B, V(-B_W / 2, -B_H / 2, 0))
    vert = [e for e in plateB.Edges if abs(e.BoundBox.ZLength - T_B) < 1e-6]
    plateB = plateB.makeFillet(4.0, vert)
    plateB = plateB.makeFillet(1.5, [e for e in plateB.Edges if abs(e.BoundBox.ZMin - T_B) < 1e-6 and abs(e.BoundBox.ZMax - T_B) < 1e-6])
    plateB = placed(plateB, oB, xB, yB, zB)
    # neck: ruled loft between a circle sunk into each plate
    cA = oA + zA * (T_A - 2.0)
    cB = oB + zB * (T_B - 2.0)
    wA = Part.Wire(Part.makeCircle(R_HUB_A, cA, zA))
    wB = Part.Wire(Part.makeCircle(R_HUB_B, cB, zB))
    neck = Part.makeLoft([wA, wB], True, True)
    body = plateA.fuse([plateB, neck]).removeSplitter()
    try:
        ring = [e for e in body.Edges if e.Curve.__class__.__name__ in ("BSplineCurve", "Ellipse", "Circle")
                and (abs(zA.dot(e.Vertexes[0].Point - oA) - T_A) < 1e-4 and abs(zA.dot(e.BoundBox.Center - oA) - T_A) < 1e-3
                     or abs(zB.dot(e.Vertexes[0].Point - oB) - T_B) < 1e-4 and abs(zB.dot(e.BoundBox.Center - oB) - T_B) < 1e-3)
                and e.Length < 2 * 3.1416 * 16.5]
        if ring:
            body = body.makeFillet(3.0, ring)
    except Exception:
        traceback.print_exc()
    # clearance holes, through both plates
    for b in BOLTS:
        p, a = V(*b["position"]), V(*b["axis"])
        d = 5.5 if b["size"] == "M5" else 6.6
        body = body.cut(Part.makeCylinder(d / 2, 40, p - a * 5, a))
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
