"""Connecting part between fixed A (square, 4x M4, below) and fixed B (round, 3x M5, above).

Layout (world mm):
  * Central column (solid, dia 12) on A's face, centred under B, rising to z=-58.
  * Bottom star on A: four 4 mm pads (M4 bearing faces) joined to the column by
    6 mm arms. It lies on A's face, so it prints flat on the bed.
  * Top: three 5 mm pads (M5 bearing faces) under B's face. Each has a short tab
    and a dia-4.8 strut running down-inward to the column at about 52 deg, which
    keeps every strut underside inside the 45 deg overhang limit and carries the
    load mainly in tension and compression.
  * Clearance holes on each bolt axis. A bent arm routes round the B3 column.
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
T_A = 4.0              # bottom pad / arm thickness (M4 grip)
T_B = 5.0              # top pad thickness (M5 grip)
COL_R = 6.0            # central column radius
COL_TOP = -58.0        # column top, where the top struts meet it
PAD_RA = 4.0           # M4 bearing pad radius
PAD_RB = 4.25          # M5 bearing pad radius
ARM_WA = 6.0           # bottom arm width
TAB_W = 6.0            # top tab width
TAB_L = 6.5            # tab length from the M5 axis toward the column
STRUT_R = 2.4          # top strut radius
STRUT_Z = Z_B - T_B / 2
HOLE_D = {"M4": 4.5, "M5": 5.5}
# column axis (xy): the M5 pattern centre
COL_XY = (-124.18164228311065, 135.11206773573574)
# arm waypoints that keep an arm clear of another bolt's column (xy)
ARM_WAYPOINTS = {"A3": [(-90.75, 114.75)]}


def xy(b):
    return (b["position"][0], b["position"][1])


def strip(p0, p1, w, z0, t):
    """Flat bar of width w from p0 to p1 (xy), from z0 up by t."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L * w / 2, dx / L * w / 2
    pts = [V(p0[0] + nx, p0[1] + ny, z0), V(p1[0] + nx, p1[1] + ny, z0),
           V(p1[0] - nx, p1[1] - ny, z0), V(p0[0] - nx, p0[1] - ny, z0)]
    pts.append(pts[0])
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, t))


def build() -> Part.Shape:
    P = COL_XY
    bolts_a = [b for b in BOLTS if b["id"].startswith("A")]
    bolts_b = [b for b in BOLTS if b["id"].startswith("B")]
    parts = []

    # ---- central column on A ----
    parts.append(Part.makeCylinder(COL_R, COL_TOP - Z_A, V(P[0], P[1], Z_A), V(0, 0, 1)))

    # ---- bottom star on A ----
    for b in bolts_a:
        q = xy(b)
        parts.append(Part.makeCylinder(PAD_RA, T_A, V(q[0], q[1], Z_A), V(0, 0, 1)))
        route = [P] + ARM_WAYPOINTS.get(b["id"], []) + [q]
        for p0, p1 in zip(route[:-1], route[1:]):
            parts.append(strip(p0, p1, ARM_WA, Z_A, T_A))

    # ---- top pads, tabs and struts on B ----
    for b in bolts_b:
        q = xy(b)
        parts.append(Part.makeCylinder(PAD_RB, T_B, V(q[0], q[1], Z_B - T_B), V(0, 0, 1)))
        dx, dy = P[0] - q[0], P[1] - q[1]
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        tab_end = (q[0] + ux * TAB_L, q[1] + uy * TAB_L)
        parts.append(strip(q, tab_end, TAB_W, Z_B - T_B, T_B))
        s = V(tab_end[0], tab_end[1], STRUT_Z)
        e = V(P[0] + ux * (COL_R - 1.0), P[1] + uy * (COL_R - 1.0), COL_TOP)
        d = e.sub(s)
        parts.append(Part.makeCylinder(STRUT_R, d.Length, s, d.normalize()))

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
