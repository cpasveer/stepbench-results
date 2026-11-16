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
    return v.multiply(1.0 / v.Length)


def frame(face):
    n = V(*face["normal"])
    n = unit(V(n.x, n.y, 0))
    t = V(-n.y, n.x, 0)
    return V(*face["origin"]), n, t


def prism(O, n, t, pts, n0, n1, zshift=0.0):
    """Prism over polygon pts=(u,z) in the (t, Z) plane, from n0 to n1 along n."""
    w = [O + t * u + V(0, 0, z) + n * n0 for u, z in pts]
    w.append(w[0])
    f = Part.Face(Part.makePolygon(w))
    return f.extrude(n * (n1 - n0))


def ring_face_prism(O, n, t, centers, half, thick):
    cs = [O + t * u + V(0, 0, z) for u, z in centers]
    cs.append(cs[0])
    w = Part.makePolygon(cs)
    outer = Part.Face(w.makeOffset2D(half, 0, False, False, True))
    inner = Part.Face(w.makeOffset2D(-half, 0, False, False, True))
    return outer.cut(inner).extrude(n * thick)


def build():
    OA, nA, tA = frame(FACES[0])
    OB, nB, tB = frame(FACES[1])
    TA, TB = 4.0, 5.0
    zA = OA.z
    # A: solid hexagonal plate around the bolt circle, V skirt to the web
    hexc = [(-30, 0), (-15, 25.98), (15, 25.98), (30, 0), (15, -25.98), (-15, -25.98)]
    hexc = [(u, z + zA) for u, z in hexc]
    solid = ring_face_prism(V(OA.x, OA.y, 0), nA, tA, hexc, 5.5, TA)
    cs = [V(OA.x, OA.y, 0) + tA * u + V(0, 0, z) for u, z in hexc]
    cs.append(cs[0])
    plateA = Part.Face(Part.Wire(Part.makePolygon(cs).makeOffset2D(5.5, 0, False, False, True))).extrude(nA * TA)
    solid = plateA
    # Lambda-shaped underside (no flat ceiling): cut the triangle under the two lower bolts
    solid = solid.cut(prism(OA, nA, tA, [(-15, -31.5), (0, -16.5), (15, -31.5), (15, -40), (-15, -40)], -1, TA + 1))
    # B: solid lower plate, two bars above
    solid = solid.fuse(prism(OB, nB, tB, [(-21, -46), (21, -46), (21, 34), (-21, 34)], 0, TB))
    for u in (-15, 15):
        solid = solid.fuse(prism(OB, nB, tB, [(u - 6, 30), (u + 6, 30), (u + 6, 46), (u - 6, 46)], 0, TB))
    # web: plan polyline strip, straight out of A then angled to B
    w = 4.0
    sBc = 3.5
    P0 = V(OA.x, OA.y, 0) + nA * TA
    P1 = V(OA.x, OA.y, 0) + nA * 46
    P2 = V(OB.x, OB.y, 0) + tB * sBc + nB * (TB / 2)
    d1 = unit(P1 - P0)
    d2 = unit(P2 - P1)
    n1 = V(-d1.y, d1.x, 0)
    n2 = V(-d2.y, d2.x, 0)
    m = unit(n1 + n2)
    mw = w / m.dot(n1)
    left = [P0 + n1 * w, P1 + m * mw, P2 + n2 * w]
    right = [P0 - n1 * w, P1 - m * mw, P2 - n2 * w]
    poly = left + right[::-1]
    poly = [V(p.x, p.y, -64) for p in poly]
    poly.append(poly[0])
    web = Part.Face(Part.makePolygon(poly)).extrude(V(0, 0, 80))
    solid = solid.fuse(web)
    for b in BOLTS:
        p = V(*b["position"])
        ax = V(*b["axis"])
        d = 5.5 if b["size"] == "M5" else 6.7
        h = Part.makeCylinder(d / 2, 12, p - ax * 2, ax)
        solid = solid.cut(h)
    return solid.removeSplitter()


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
