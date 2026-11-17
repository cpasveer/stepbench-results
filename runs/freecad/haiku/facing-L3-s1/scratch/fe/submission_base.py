"""Connecting part between fixed A (square, 4x M4, below) and fixed B (round, 3x M5, above).

Layout (world mm):
  * Central column: a solid cone, 12.5 mm radius on A's face tapering to 6 mm at
    B's face, centred under B's bolt pattern. Bending stiffness comes from the
    column, so its base carries most of the load.
  * Top: three M5 bearing pads (5 mm, under B's face). Each pad joins the column
    through a 6 mm wedge gusset whose underside slopes at 50 deg, so no
    horizontal ceiling is wider than the bearing ring.
  * Bottom: four M4 bearing pads (4 mm, on A's face) joined to the column base by
    6 mm arms that lie flat on A and print on the bed. One arm bends round the
    B3 column.
  * Clearance holes on every bolt axis, through each bolt's own pad only.
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
T_A = 4.0              # bottom pad / arm thickness (M4 grip >= 3)
T_B = 5.0              # top pad thickness (M5 grip >= 3.75)
COL_R_BASE = 12.5      # column radius on A's face
COL_R_TOP = 6.0        # column radius on B's face
PAD_RA = 4.5           # M4 pad radius
PAD_RB = 4.3           # M5 pad radius (covers the 4.25 mm head ring)
ARM_W = 6.0            # bottom arm width
WEDGE_W = 5.0            # top gusset width
GUSSET_DEG = 50.0      # gusset underside slope (overhang limit 45 deg)
GUSSET_S0 = 4.8        # gusset underside starts this far from the M5 axis (clear of head envelope)
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


def gusset(q, P):
    """Wedge from the M5 pad to the column. Top flat at Z_B, underside flat near the
    pad and sloping down at GUSSET_DEG toward the column. Built in the vertical
    plane through q and P, then given width WEDGE_W."""
    dx, dy = P[0] - q[0], P[1] - q[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy * WEDGE_W / 2, ux * WEDGE_W / 2
    sc = L - 3.0                                     # gusset runs 3 mm into the column (no stubs)
    drop = (sc - GUSSET_S0) * math.tan(math.radians(GUSSET_DEG))
    prof = [(0.0, Z_B), (sc, Z_B), (sc, Z_B - T_B - drop), (GUSSET_S0, Z_B - T_B), (0.0, Z_B - T_B)]

    def pt(s, z, sg):
        return V(q[0] + ux * s + sg * nx, q[1] + uy * s + sg * ny, z)
    pts = [pt(s, z, -1) for s, z in prof]
    pts.append(pts[0])
    face = Part.Face(Part.makePolygon(pts))
    return face.extrude(V(2 * nx, 2 * ny, 0))


def build() -> Part.Shape:
    P = COL_XY
    bolts_a = [b for b in BOLTS if b["id"].startswith("A")]
    bolts_b = [b for b in BOLTS if b["id"].startswith("B")]
    parts = []

    # ---- central column (cone) from A's face to B's face ----
    parts.append(Part.makeCone(COL_R_BASE, COL_R_TOP, Z_B - Z_A, V(P[0], P[1], Z_A), V(0, 0, 1)))

    # ---- bottom star on A ----
    for b in bolts_a:
        q = xy(b)
        parts.append(Part.makeCylinder(PAD_RA, T_A, V(q[0], q[1], Z_A), V(0, 0, 1)))
        route = [P] + ARM_WAYPOINTS.get(b["id"], []) + [q]
        for p0, p1 in zip(route[:-1], route[1:]):
            parts.append(strip(p0, p1, ARM_W, Z_A, T_A))

    # ---- top pads and gussets on B ----
    for b in bolts_b:
        q = xy(b)
        parts.append(Part.makeCylinder(PAD_RB, T_B, V(q[0], q[1], Z_B - T_B), V(0, 0, 1)))
        parts.append(gusset(q, P))

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
