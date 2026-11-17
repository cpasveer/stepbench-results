"""Connecting part for flange-L2-s1: a round pad on face A, a rectangular pad
on face B, and a bridge between them. Written to output.step next to this file.

    freecadcmd submission.py             # writes output.step
    JOINTBENCH_MAX_CHECKS=6 uv run --project ../../kit jointbench verify . --by haiku --tool freecad
"""
import json
import os
import sys
import traceback

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = {b["id"]: b for b in POSE["bolts"]}
V = App.Vector

Z = V(0, 0, 1)

# ---- parameters (mm) -------------------------------------------------------
T_A = 8.0            # pad thickness under the M5 heads (grip 3.75..25)
R_PAD_A = 35.0       # round pad radius (face outline 37.25, 1 mm inside rule)
T_B = 8.0            # pad thickness under the M6 heads (grip 4.5..25)
PAD_B = (42.0, 90.0)  # rectangular pad (face outline 46 x 96)
PAD_B_R = 5.0        # corner radius of the rectangular pad
HOLE_A = 5.5         # M5 clearance (5.3..5.8)
HOLE_B = 6.6         # M6 clearance (6.4..7.0)
BRIDGE_R = 8.0       # bridge radius


def frame_matrix(origin, ex, ey, ez):
    m = App.Matrix()
    cols = [ex, ey, ez]
    for j, c in enumerate(cols):
        setattr(m, "A%d%d" % (1, j + 1), c.x)
        setattr(m, "A%d%d" % (2, j + 1), c.y)
        setattr(m, "A%d%d" % (3, j + 1), c.z)
    m.A14 = origin.x
    m.A24 = origin.y
    m.A34 = origin.z
    return m


def build_pad_a():
    fa = FACES["A"]
    o = V(*fa["origin"])
    n = V(*fa["normal"]).normalize()
    return Part.makeCylinder(R_PAD_A, T_A, o, n)


def build_pad_b():
    fb = FACES["B"]
    o = V(*fb["origin"])
    n = V(*fb["normal"]).normalize()
    # local frame: x = Z x n (the horizontal in-plane axis), y = Z, z = n
    ey = Z
    ez = n
    ex = ey.cross(ez)  # ensures right-handed frame
    ex = V(ex.x, ex.y, ex.z).normalize()
    box = Part.makeBox(PAD_B[0], PAD_B[1], T_B, V(-PAD_B[0] / 2, -PAD_B[1] / 2, 0))
    edges = [e for e in box.Edges
             if abs(e.Vertexes[0].Point.z - e.Vertexes[1].Point.z) > 1e-6]
    box = box.makeFillet(PAD_B_R, edges)
    m = frame_matrix(o, ex, ey, ez)
    return box.transformShape(m)


def build_bridge():
    fa, fb = FACES["A"], FACES["B"]
    na = V(*fa["normal"]).normalize()
    nb = V(*fb["normal"]).normalize()
    q1 = V(*fa["origin"]) + na * (T_A / 2)
    q2 = V(*fb["origin"]) + nb * (T_B / 2)
    d = q2 - q1
    return Part.makeCylinder(BRIDGE_R, d.Length, q1, d.normalize())


def hole(p, n, dia, depth_back, depth_front):
    base = p - n * depth_back
    return Part.makeCylinder(dia / 2, depth_back + depth_front, base, n)


def build():
    na = V(*FACES["A"]["normal"]).normalize()
    nb = V(*FACES["B"]["normal"]).normalize()
    body = build_pad_a().fuse(build_pad_b()).fuse(build_bridge())
    for bid, b in BOLTS.items():
        p = V(*b["position"])
        if b["size"] == "M5":
            n, dia = na, HOLE_A
            h = hole(p, n, dia, 1.0, T_A)
        else:
            n, dia = nb, HOLE_B
            h = hole(p, n, dia, 1.0, T_B)
        body = body.cut(h)
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "valid", shape.isValid())
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
