"""Connecting part between fixed A (square, 4x M4, below) and fixed B (round, 3x M5, above).

Layout (all world mm):
  * Bottom star on A: four pads (4 mm thick, the M4 bearing faces) and flat
    arms of 6 mm width running in from each pad to a central hub.
  * Hub: a short solid column from A's face up to z = -85.
  * Top: three M5 pads (5 mm thick) under B's face. Each pad has a small tab
    and a slender leg (dia 4.8) that runs steeply (about 48 deg or more) down to
    the hub, so the top of the part needs no horizontal ceilings beyond the
    bolt-head bearing rings and the legs print without support.
  * Clearance holes on every bolt axis; heads and tool access are kept free
    by construction and checked locally before submission.
Run:  freecadcmd submission.py   (writes output.step next to this file)
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
POSE = TASK["poses"][0]["resolved"]
FACES = POSE["mounting_faces"]
BOLTS = POSE["bolts"]
V = App.Vector

Z_A = -94.0            # A mounting face (A body below it)
Z_B = -25.5            # B mounting face (B body above it)
T_A = 4.0              # bottom star thickness (M4 grip)
T_B = 5.0              # top pad thickness (M5 grip)
HUB_R = 7.0
HUB_TOP = -85.0
LEG_R = 2.4
LEG_Z = Z_B - T_B / 2  # leg axis height at the top pads
PAD_RA = 4.0           # M4 bearing pad radius (ring to 3.5 mm, 1 mm inside A's edge)
PAD_RB = 4.25          # M5 bearing pad radius (ring to 4.25 mm)
ARM_WA = 6.0           # bottom arm width
TAB_W = 6.0            # top tab width
TAB_L = 9.0            # top tab length from B bolt axis toward the hub
LEG_START = 7.0        # leg starts inside its tab, clear of the head envelope
HOLE_D = {"M4": 4.5, "M5": 5.5}
# hub centre (xy), chosen so no arm or leg crosses another bolt's column
HUB_XY = (-122.0, 114.5)
# bottom arms that must bend round another bolt's column (xy)
ARM_WAYPOINTS = {"A3": [(-95.0, 108.0)]}


def xy(b):
    return (b["position"][0], b["position"][1])


def arm_face(p0, p1, w):
    """Closed flat strip (xy) of width w from p0 to p1."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L * w / 2, dx / L * w / 2
    pts = [V(p0[0] + nx, p0[1] + ny, 0), V(p1[0] + nx, p1[1] + ny, 0),
           V(p1[0] - nx, p1[1] - ny, 0), V(p0[0] - nx, p0[1] - ny, 0)]
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts))


def build() -> Part.Shape:
    P = HUB_XY
    bolts_a = [b for b in BOLTS if b["id"].startswith("A")]
    bolts_b = [b for b in BOLTS if b["id"].startswith("B")]
    parts = []

    # ---- hub ----
    parts.append(Part.makeCylinder(HUB_R, HUB_TOP - Z_A, V(P[0], P[1], Z_A), V(0, 0, 1)))

    # ---- bottom star on A ----
    for b in bolts_a:
        q = xy(b)
        parts.append(Part.makeCylinder(PAD_RA, T_A, V(q[0], q[1], Z_A), V(0, 0, 1)))
        route = [P] + ARM_WAYPOINTS.get(b["id"], []) + [q]
        for p0, p1 in zip(route[:-1], route[1:]):
            parts.append(arm_face(p0, p1, ARM_WA).extrude(V(0, 0, T_A)).translated(V(0, 0, Z_A)))

    # ---- top pads, tabs and legs on B ----
    z_top_pad = Z_B - T_B
    for b in bolts_b:
        q = xy(b)
        parts.append(Part.makeCylinder(PAD_RB, T_B, V(q[0], q[1], z_top_pad), V(0, 0, 1)))
        ux, uy = (P[0] - q[0]), (P[1] - q[1])
        L = math.hypot(ux, uy)
        ux, uy = ux / L, uy / L
        tab_end = (q[0] + ux * TAB_L, q[1] + uy * TAB_L)
        parts.append(arm_face(q, tab_end, TAB_W).extrude(V(0, 0, T_B)).translated(V(0, 0, z_top_pad)))
        s = (q[0] + ux * LEG_START, q[1] + uy * LEG_START)
        leg_top = V(s[0], s[1], LEG_Z)
        leg_bot = V(P[0], P[1], HUB_TOP - 2.0)   # ends inside the hub
        d = leg_bot.sub(leg_top)
        parts.append(Part.makeCylinder(LEG_R, d.Length, leg_top, d.normalize()))

    body = parts[0]
    for p in parts[1:]:
        body = body.fuse(p)
    body = body.removeSplitter()

    # ---- clearance holes: each bolt axis, through its own pad only ----
    for b in BOLTS:
        q = xy(b)
        dia = HOLE_D[b["size"]]
        if b["axis"][2] > 0:    # A bolts come up through the bottom pad
            base = V(q[0], q[1], Z_A - 1.0)
            h = T_A + 2.0
        else:                    # B bolts come down through the top pad
            base = V(q[0], q[1], Z_B + 1.0)
            h = T_B + 2.0
        body = body.cut(Part.makeCylinder(dia / 2, h, base, V(0, 0, b["axis"][2])))
    return body.removeSplitter()


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    shape.exportStep(OUT)
    print("wrote", OUT, "volume", round(shape.Volume, 1))
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
