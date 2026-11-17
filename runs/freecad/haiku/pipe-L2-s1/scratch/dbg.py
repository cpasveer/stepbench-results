"""Connecting bracket between pipe A (4x M5 radial) and square mount B (4x M4).

Design (all dimensions in mm):
- Saddle: a two-flat wedge bearing on the 68 mm pipe. The flats are normal to
  the two bolt pairs, so each M5 head sits square on a flat at 8 mm grip.
  Inner face is the pipe cylinder (exact contact, no overlap).
- Web: a tapered prism from the saddle crest to the B mount plate.
- B plate: 34 x 34 x 6 mm pad on the mounting face (inside the 1 mm inset),
  so each M4 head sits on a flat 6 mm grip.
- Holes: 5.5 mm clearance for M5, 4.5 mm for M4, along each bolt axis.
Everything is cut to the pipe cylinder and to the B face half-space so that
nothing overlaps the fixed parts.
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
PL = POSE["placements"]
BOLTS = {b["id"]: b for b in POSE["resolved"]["bolts"]}
V = App.Vector


def unit(v):
    l = v.Length
    return V(v.x / l, v.y / l, v.z / l)


def frame(xa, ya, za, origin):
    """Matrix taking local (x,y,z) to world with the given axes and origin."""
    return App.Matrix(
        xa.x, ya.x, za.x, origin.x,
        xa.y, ya.y, za.y, origin.y,
        xa.z, ya.z, za.z, origin.z,
        0, 0, 0, 1,
    )


# ---- pipe A frame (local x = pipe axis, y/z across the section)
pa = PL["A"]
RA = pa["rotation"]
XA = V(RA[0][0], RA[1][0], RA[2][0])
YA = V(RA[0][1], RA[1][1], RA[2][1])
ZA = V(RA[0][2], RA[1][2], RA[2][2])
OA = V(*pa["origin"])
MA = frame(XA, YA, ZA, OA)

# ---- B mount frame (local z = outward normal of the mounting face)
pb = PL["B"]
RB = pb["rotation"]
XB = V(RB[0][0], RB[1][0], RB[2][0])
YB = V(RB[0][1], RB[1][1], RB[2][1])
ZB = V(RB[0][2], RB[1][2], RB[2][2])
OB = V(*pb["origin"])
MB = frame(XB, YB, ZB, OB)

PIPE_R = 34.0
HEAD_A = 42.0          # head bearing plane of M5 heads: pipe radius + grip
GRIP_A = HEAD_A - PIPE_R   # 8 mm
GRIP_B = 6.0           # B plate thickness under M4 heads
SADDLE_HALF_X = 24.0   # saddle half length along pipe axis
PLATE_HALF = 17.0      # B plate half width (inset 1 mm from 38 mm face)
CLR_A = 5.5            # M5 clearance diameter
CLR_B = 4.5            # M4 clearance diameter
WEB_TOP_X = 6.0        # web extent toward +pipe axis (from the axis)


def saddle_local():
    """Two-flat wedge in local coords, pipe cylinder removed."""
    # bolt directions in pipe section: angle measured from +z toward -y / +y
    t1 = math.radians(-35.0)
    t2 = math.radians(35.0)
    a = math.radians(50.0)
    h = HEAD_A
    def pt(th, r, x):
        return V(x, r * math.sin(th), r * math.cos(th))
    x0 = -SADDLE_HALF_X
    pa_ = pt(-a, h / math.cos(math.radians(15)), x0)
    crest = pt(0.0, h / math.cos(math.radians(35)), x0)
    pb_ = pt(a, h / math.cos(math.radians(15)), x0)
    o = V(x0, 0, 0)
    wire = Part.makePolygon([o, pa_, crest, pb_, o])
    face = Part.Face(wire)
    solid = face.extrude(V(2 * SADDLE_HALF_X, 0, 0))
    cyl = Part.makeCylinder(PIPE_R, 200, V(-100, 0, 0), V(1, 0, 0))
    return solid.cut(cyl)


def build():
    # --- saddle on the pipe, in pipe-local coordinates, then to world
    sad = saddle_local().transformShape(MA)

    # --- B plate: sits on the mounting face, behind it is the B part
    plate = Part.makeBox(2 * PLATE_HALF, 2 * PLATE_HALF, GRIP_B,
                         V(-PLATE_HALF, -PLATE_HALF, 0))
    # 45 deg chamfer on the -x top edge: that face would otherwise hang at 40 deg
    # when the part stands on the saddle end
    ch = GRIP_B
    cut_pts = [V(-PLATE_HALF, -20, 0), V(-PLATE_HALF + ch, -20, GRIP_B),
               V(-30, -20, GRIP_B), V(-30, -20, 0)]
    cut_face = Part.Face(Part.makePolygon(cut_pts + [cut_pts[0]]))
    cutter = cut_face.extrude(V(0, 40, 0))
    plate = plate.cut(cutter)
    plate = plate.transformShape(MB)

    # --- web from saddle crest to B plate: a thin wall whose faces stay
    # vertical when the part stands on the saddle end (pipe axis up)
    S = OA + ZA * (PIPE_R + 6.0)                 # inside the saddle, at the crest
    T = OB + XB * (-10.0) + ZB * 3.0             # inside the B plate, toward its -x edge
    v = T - S
    w = unit(v)
    a = unit(w.cross(XA))                        # wall normal: horizontal across the wall
    L = v.Length
    t_web = 7.0
    web_top = WEB_TOP_X                           # keep the web over the B plate's footprint
    box = Part.makeBox(t_web, web_top + SADDLE_HALF_X, L)   # thickness, height (pipe axis), length
    base = S - a * (t_web / 2) - XA * SADDLE_HALF_X
    col = box.transformShape(frame(a, XA, w, base))
    # keep the web's B-end inside the mounting face's 1 mm inset line
    # (B-frame x = -17.5 on the face). The cutting plane contains the pipe-axis
    # direction, so its face is vertical when the part stands on the saddle end.
    bad = Part.Face(Part.makePolygon([
        V(-17.5, -300, 0), V(-17.5 + 1.2 * 400, -300, 400),
        V(-600, -300, 400), V(-600, -300, 0), V(-17.5, -300, 0)]))
    bad = bad.extrude(V(0, 600, 0)).transformShape(MB)
    col = col.cut(bad)

    print("DBG sad", sad.Volume, sad.isValid(), "plate", plate.Volume, "col", col.Volume, "bad", bad.Volume)
    body = sad.fuse(plate).fuse(col)
    # never inside the pipe, never behind the B face
    body = body.cut(Part.makeCylinder(PIPE_R, 400, OA - XA * 200, XA))
    half = Part.makeBox(400, 400, 400, V(-200, -200, 0)).transformShape(MB)
    body = body.common(half)

    # --- clearance holes along each bolt axis
    for bid, bolt in BOLTS.items():
        c = V(*bolt["position"])
        d = V(*bolt["axis"])
        if bolt["size"] == "M5":
            dia = CLR_A
        else:
            dia = CLR_B
        cyl = Part.makeCylinder(dia / 2, 80, c - d * 40, d)
        body = body.cut(cyl)
    try:
        return body.removeSplitter()
    except Exception:
        return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        try:
            shape = shape.Solids[0].removeSplitter()
        except Exception:
            shape = shape.Solids[0]
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1), "valid", shape.isValid())
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
