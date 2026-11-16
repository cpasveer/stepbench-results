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


def lineplane(P, d, O, n, dist):
    """Point where the line P + s*d meets the plane (X-O).n = dist."""
    s = (dist - (P - O).dot(n)) / d.dot(n)
    return P + d * s


def quad_at(P0, P2, l, h, OA, nA, dA, OB, nB, dB, z):
    d = unit(P2 - P0)
    c = []
    for sgn in (-1, 1):
        a = lineplane(P0 + l * (sgn * h), d, OA, nA, dA)
        b = lineplane(P2 + l * (sgn * h), d, OB, nB, dB)
        c.append((a, b))
    (a1, b1), (a2, b2) = c
    pts = [a1, a2, b2, b1]
    return [V(p.x, p.y, z) for p in pts]


def poly_wire(pts):
    return Part.makePolygon(pts + [pts[0]])


def build():
    OA, nA, tA = frame(FACES[0])
    OB, nB, tB = frame(FACES[1])
    TA, TB = 6.0, 5.0
    zA = OA.z
    R = 3 ** 0.5 * 15
    # A: solid hexagonal plate around the bolt circle, lambda underside
    hexc = [(-30, 0), (-15, R), (15, R), (30, 0), (15, -R), (-15, -R)]
    hexc = [(u, z + zA) for u, z in hexc]
    cs = [V(OA.x, OA.y, 0) + tA * u + V(0, 0, z) for u, z in hexc]
    cs.append(cs[0])
    off = Part.makePolygon(cs).makeOffset2D(5.5, 0, False, False, True)
    solid = Part.Face(Part.Wire(off)).extrude(nA * TA)
    zb = -(R + 5.5)
    sl = 1.1
    rise = 0.7  # the keel edges climb along the thickness so each layer is supported

    def cut_sheared(pts):
        nonlocal solid
        w = [OA + tA * u + V(0, 0, z - rise) - nA * 1.0 for u, z in pts]
        w.append(w[0])
        f = Part.Face(Part.makePolygon(w))
        solid = solid.cut(f.extrude(nA * (TA + 2) + V(0, 0, rise * (TA + 2))))

    cut_sheared([(-15, zb), (0, zb + 15 * sl), (15, zb), (15, zb - 9), (-15, zb - 9)])
    cut_sheared([(-15, zb), (-40, zb + 25 * sl), (-40, zb - 9), (-15, zb - 9)])
    cut_sheared([(15, zb), (40, zb + 25 * sl), (40, zb - 9), (15, zb - 9)])
    # B: solid lower plate, two bars above
    solid = solid.fuse(prism(OB, nB, tB, [(-21, -46), (21, -46), (21, 34), (-21, 34)], 0, TB))
    for u in (-15, 15):
        solid = solid.fuse(prism(OB, nB, tB, [(u - 6, 30), (u + 6, 30), (u + 6, 46), (u - 6, 46)], 0, TB))
    # connecting member: narrow stem from the bed, 45-degree skirt, wide flange
    cA, sB = CA, SB
    P0 = V(OA.x, OA.y, 0) + tA * cA + nA * TA
    P2 = V(OB.x, OB.y, 0) + tB * sB + nB * (TB / 2)
    d = unit(P2 - P0)
    l = V(-d.y, d.x, 0)
    hs, hf = 3.0, 11.0
    z_stem_top, z_fl_bot, z_fl_top = -13.0, -4.0, 14.0
    args = (P0, P2, l)
    qa = (OA, nA, TA, OB, nB, TB / 2)
    qaf = (OA, nA, TA - 1.0, OB, nB, TB / 2)
    stem = Part.Face(poly_wire(quad_at(*args, hs, *qa, -64))).extrude(V(0, 0, z_stem_top + 64))
    q1 = quad_at(*args, hs, *qa, z_stem_top)
    q2 = quad_at(*args, hf, *qa, z_fl_bot)
    faces = [Part.Face(poly_wire(q1)), Part.Face(poly_wire(q2))]
    for k in range(4):
        faces.append(Part.Face(poly_wire([q1[k], q1[(k + 1) % 4], q2[(k + 1) % 4], q2[k]])))
    skirt = Part.Solid(Part.Shell(faces))
    if skirt.Volume < 0:
        skirt.reverse()
    flange = Part.Face(poly_wire(quad_at(*args, hf, *qaf, z_fl_bot))).extrude(V(0, 0, z_fl_top - z_fl_bot))
    for part in (stem, skirt, flange):
        solid = solid.fuse(part)
    for b in BOLTS:
        p = V(*b["position"])
        ax = V(*b["axis"])
        dd = 5.5 if b["size"] == "M5" else 6.7
        h = Part.makeCylinder(dd / 2, 14, p - ax * 2, ax)
        solid = solid.cut(h)
    return solid.removeSplitter()


CA, SB = -6, 6


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
