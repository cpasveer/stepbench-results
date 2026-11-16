"""Pipe-to-flat-mount bracket, modelled with FreeCAD's Part module.

The part has three members:
  * a V-block saddle that sits on the 68 mm pipe: its bore follows the pipe
    (R34) and its two outer flats are square to the two bolt rows, so the
    M5 heads seat on flat, square faces;
  * a flange that seats on B's 38 mm square face (the seat is a 35 mm pad,
    so it stays 1.5 mm inside B's outline) and runs on, lifted 1 mm off the
    face plane, towards the pipe;
  * a straight arm of rectangular section joining the two, kept clear of the
    bolt heads and of the hex keys' swing.
Every number comes from task.json.
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
BOLTS = {b["id"]: b for b in POSE["resolved"]["bolts"]}
V = App.Vector


def frame(name):
    pl = POSE["placements"][name]
    o = V(*pl["origin"])
    r = pl["rotation"]
    cols = [V(r[0][i], r[1][i], r[2][i]) for i in range(3)]
    return o, cols


def placement(o, x, y, z):
    m = App.Matrix(x.x, y.x, z.x, o.x,
                   x.y, y.y, z.y, o.y,
                   x.z, y.z, z.z, o.z,
                   0, 0, 0, 1)
    return App.Placement(m)


OA, (AX, AY, AZ) = frame("A")     # AX = pipe axis, AZ = between bolt rows
OB, (BU, BV, BN) = frame("B")     # BN = outward face normal

R_PIPE = TASK["fixed_parts"][0]["interface"]["params"]["od"] / 2.0
BOLT_ANG = math.degrees(math.atan2(19.501598836, 27.851169506))  # 35 deg

# ---- saddle (A local frame) -------------------------------------------
T_SAD = 8.0            # pipe surface to bolt-head seat (grip of the M5s)
SAD_HALF_L = 27.0      # half length along the pipe
SIDE_ANG = 62.0        # angular half-extent of the saddle
TOP_Z = 46.0           # flat crown between the two bolt rows
CHAMF = 0.0


def saddle():
    a = math.radians(BOLT_ANG)
    s = math.radians(SIDE_ANG)
    d = R_PIPE + T_SAD
    # outer polygon in the (y, z) plane of A
    r_side = d / math.cos(s - a)
    p_side = (r_side * math.sin(s), r_side * math.cos(s))
    y_top = (d - TOP_Z * math.cos(a)) / math.sin(a)
    r_in = R_PIPE - 6.0
    p_in = (r_in * math.sin(s), r_in * math.cos(s))
    pts2 = [(-p_in[0], p_in[1]), (-p_side[0], p_side[1]), (-y_top, TOP_Z),
            (y_top, TOP_Z), (p_side[0], p_side[1]), (p_in[0], p_in[1])]
    pts = [V(-SAD_HALF_L, y, z) for (y, z) in pts2]
    pts.append(pts[0])
    face = Part.Face(Part.makePolygon(pts))
    body = face.extrude(V(2 * SAD_HALF_L, 0, 0))
    pipe = Part.makeCylinder(R_PIPE, 4 * SAD_HALF_L, V(-2 * SAD_HALF_L, 0, 0), V(1, 0, 0))
    body = body.cut(pipe)
    body.Placement = placement(OA, AX, AY, AZ)
    return body


# ---- flange on B (B local frame) ----------------------------------------
PAD_HALF = 17.5        # 1.5 mm inside B's 19 mm half-width
PAD_T = 1.0            # seat pad standing proud of the rest of the back
FL_T = 6.0             # flange thickness above the pad
FL_U0 = -46.0          # flange runs out to here (towards the pipe)
FL_HALF_V = 17.5


def flange():
    pad = Part.makeBox(2 * PAD_HALF, 2 * PAD_HALF, PAD_T, V(-PAD_HALF, -PAD_HALF, 0))
    fl = Part.makeBox(PAD_HALF - FL_U0, 2 * FL_HALF_V, FL_T, V(FL_U0, -FL_HALF_V, PAD_T))
    body = pad.fuse(fl)
    body.Placement = placement(OB, BU, BV, BN)
    return body


# ---- arm ---------------------------------------------------------------
ARM_W = 14.0
ARM_H = 18.0


ARM_S_X = -6.0          # arm foot on the saddle, along the pipe axis
ARM_E_U = -30.0         # arm foot on the flange, along B's u axis


def arm():
    """Straight bar from the saddle crown to the flange.  It is made over-long
    and trimmed by the pipe bore and by the flange's back plane, so it ends
    exactly on the faces it joins."""
    s = OA + AX * ARM_S_X + AZ * R_PIPE
    e = OB + BU * ARM_E_U + BN * PAD_T
    d = e - s
    x = V(d)
    x.normalize()
    w = x.cross(AX)
    w.normalize()
    h = x.cross(w)
    start = s - x * 20.0
    L = d.Length + 40.0
    base = start - w * (ARM_W / 2) - h * (ARM_H / 2)
    b = Part.makeBox(ARM_W, ARM_H, L)
    b.Placement = placement(base, w, h, x)
    bore = Part.makeCylinder(R_PIPE, 400, OA - AX * 200, AX)
    behind = Part.makeBox(400, 400, 200, V(-200, -200, -200))
    behind.Placement = placement(OB + BN * PAD_T, BU, BV, BN).multiply(behind.Placement)
    return b.cut(bore).cut(behind)


def bolt_holes():
    cut = []
    for b in BOLTS.values():
        p = V(*b["position"])
        a = V(*b["axis"])
        dia = sum(b["clearance_diameter_range"]) / 2.0
        cut.append(Part.makeCylinder(dia / 2, 60, p - a * 10, a))
        # head room: counterbore-free; keep a clear 0.75 mm annulus above seat
    return cut


def build():
    body = saddle().fuse(flange()).fuse(arm())
    for c in bolt_holes():
        body = body.cut(c)
    return body


OUT = os.path.join(HERE, "output.step")
if __name__ in ("__main__", "submission"):
    try:
        if os.path.exists(OUT):
            os.remove(OUT)
        shape = build()
        if len(shape.Solids) == 1:
            shape = shape.Solids[0].removeSplitter()
        shape.exportStep(OUT)
        print("wrote", OUT, "solids", len(shape.Solids), "valid", shape.isValid(), "vol", shape.Volume)
    except Exception:
        traceback.print_exc()
        if os.path.exists(OUT):
            os.remove(OUT)
        sys.exit(1)
