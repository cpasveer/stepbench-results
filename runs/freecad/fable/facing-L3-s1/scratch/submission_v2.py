"""Connector for task facing-L3-s1, as a FreeCAD script.

    freecadcmd submission.py        # writes output.step

Concept: a side-printed channel bracket.  Two thin upright webs run along one
side of A's bolt square; each carries one or two bolts of B on a pad at its top
and two bolts of A on tabs at its foot.  A thin end panel closes the channel
at one end: it braces the webs sideways and carries B's third bolt.  A window
in the panel lets the hex key of that bolt swing.

The part is printed standing on the end panel (the plane y = Y_BED in A's
local frame, build direction +y).  Webs, tabs and pads are then all vertical
walls; edges that grow outwards while printing do so at OVERHANG degrees off
the build axis, so nothing needs support.

Everything is modelled in A's local frame (x, y along A's bolt square, z up
from A's mounting face) and moved to world coordinates at the end.
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
BOLTS = POSE["resolved"]["bolts"]
PL_A = POSE["placements"]["A"]
PL_B = POSE["placements"]["B"]
V = App.Vector

# ---- local frame of A -------------------------------------------------------
OA = PL_A["origin"]
RA = PL_A["rotation"]            # world = RA . local + OA


def to_local(p):
    d = [p[i] - OA[i] for i in range(3)]
    return [sum(RA[j][i] * d[j] for j in range(3)) for i in range(3)]


H = to_local(PL_B["origin"])[2]                    # gap between the faces
CB = to_local(PL_B["origin"])[:2]                  # centre of B's round face
LB = {b["id"]: to_local(b["position"]) for b in BOLTS}

# ---- design parameters (mm) -------------------------------------------------
T_A = 3.2            # foot tabs (grip of the M4 bolts)
T_B = 3.9            # head pads (grip of the M5 bolts)
T_WEB = 0.9          # webs and end panel: two perimeters
D_A = 4.5            # M4 clearance hole
D_B = 5.5            # M5 clearance hole
OVERHANG = 43.0      # steepest growth off the build axis, degrees
SLOPE = math.tan(math.radians(OVERHANG))
Z_B = H - T_B

Y_BED = -35.0                    # print bed plane; the part is built towards +y
Y_PANEL = Y_BED + T_WEB          # inner face of the end panel
WA_O, WA_I = -25.9, -25.0        # web A: outer / inner face (clears A1, A4 heads)
WB_I, WB_O = 17.75, 18.65        # web B: inner / outer face (clears the B3 head)
YA_END = 33.8                    # far end of web A (and of the A4 tab)
YB_END = 35.3                    # far end of web B (and of the A3 tab)
YA_SEAT, YB_SEAT = -29.5, -27.5  # where the web tops reach B's face
X_TAB = 33.8                     # outer edge of the foot tabs (|x|)
Y_TAB = -26.2                    # end of the two tabs on the bed (A1, A2)
Y_A4, Y_A3 = 20.0, 12.5          # where the far tabs start to grow out of the webs
# B1 pad: stands on the bed, on top of the end panel
B1_X0, B1_X1, B1_X2, B1_YEND = -6.6, -1.5, 2.3, -24.6
# B2 pad: grows out of web A
B2_Y0, B2_XO, B2_YEND = 2.5, -41.2, 25.0
# B3 pad: grows out of web B
B3_Y0, B3_XO, B3_YEND = 14.4, 27.8, 30.25
# key window in the end panel, in front of B1
WIN_X0, WIN_X1, WIN_Z0, WIN_Z1 = -11.2, 6.9, 27.0, 52.0


def prism_x(yz, x0, x1):
    """Polygon in (y, z), extruded from x0 to x1."""
    pts = [V(x0, y, z) for y, z in yz]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(x1 - x0, 0, 0))


def prism_y(xz, y0, y1):
    pts = [V(x, y0, z) for x, z in xz]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, y1 - y0, 0))


def prism_z(xy, z0, z1):
    pts = [V(x, y, z0) for x, y in xy]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, 0, z1 - z0))


def build_local():
    parts = []
    # webs: flush with the panel top at the bed, then rising to B's face
    parts.append(prism_x([(Y_BED, 0), (Y_BED, Z_B), (Y_PANEL, Z_B), (YA_SEAT, H),
                          (YA_END, H), (YA_END, 0)], WA_O, WA_I))
    parts.append(prism_x([(Y_BED, 0), (Y_BED, Z_B), (Y_PANEL, Z_B), (YB_SEAT, H),
                          (YB_END, H), (YB_END, 0)], WB_I, WB_O))
    # end panel on the bed
    parts.append(prism_y([(WA_O, 0), (WB_O, 0), (WB_O, Z_B), (WA_O, Z_B)], Y_BED, Y_PANEL))
    # foot tabs on the bed (A1, A2)
    parts.append(prism_z([(-X_TAB, Y_BED), (WA_I, Y_BED), (WA_I, Y_TAB), (-X_TAB, Y_TAB)], 0, T_A))
    parts.append(prism_z([(WB_I, Y_BED), (X_TAB, Y_BED), (X_TAB, Y_TAB), (WB_I, Y_TAB)], 0, T_A))
    # foot tabs at the far end (A4, A3): grow out of the webs
    parts.append(prism_z([(WA_O, Y_A4), (-X_TAB, Y_A4 + (X_TAB + WA_O) / SLOPE), (-X_TAB, YA_END),
                          (WA_I, YA_END), (WA_I, Y_A4)], 0, T_A))
    parts.append(prism_z([(WB_O, Y_A3), (X_TAB, Y_A3 + (X_TAB - WB_O) / SLOPE), (X_TAB, YB_END),
                          (WB_I, YB_END), (WB_I, Y_A3)], 0, T_A))
    # head pads
    parts.append(prism_z([(B1_X0, Y_BED), (B1_X1, Y_BED), (B1_X2, Y_BED + (B1_X2 - B1_X1) / SLOPE),
                          (B1_X2, B1_YEND), (B1_X0, B1_YEND)], Z_B, H))
    parts.append(prism_z([(WA_O, B2_Y0), (B2_XO, B2_Y0 + (WA_O - B2_XO) / SLOPE), (B2_XO, B2_YEND),
                          (WA_I, B2_YEND), (WA_I, B2_Y0)], Z_B, H))
    parts.append(prism_z([(WB_O, B3_Y0), (B3_XO, B3_Y0 + (B3_XO - WB_O) / SLOPE), (B3_XO, B3_YEND),
                          (WB_I, B3_YEND), (WB_I, B3_Y0)], Z_B, H))
    body = parts[0]
    for s in parts[1:]:
        body = body.fuse(s)

    cutters = [prism_y([(WIN_X0, WIN_Z0), (WIN_X1, WIN_Z0), (WIN_X1, WIN_Z1), (WIN_X0, WIN_Z1)],
                       Y_BED - 1, Y_PANEL + 1)]
    for b in BOLTS:
        x, y, z = LB[b["id"]]
        if b["id"].startswith("A"):
            cutters.append(Part.makeCylinder(D_A / 2, T_A + 2, V(x, y, -1), V(0, 0, 1)))
        else:
            cutters.append(Part.makeCylinder(D_B / 2, T_B + 2, V(x, y, Z_B - 1), V(0, 0, 1)))
    for c in cutters:
        body = body.cut(c)
    return body.removeSplitter()


def build() -> Part.Shape:
    s = build_local()
    mat = App.Matrix(RA[0][0], RA[0][1], RA[0][2], OA[0],
                     RA[1][0], RA[1][1], RA[1][2], OA[1],
                     RA[2][0], RA[2][1], RA[2][2], OA[2],
                     0, 0, 0, 1)
    s.transformShape(mat)          # rigid move: planes stay planes
    return s


if __name__ == "__main__" or True:
    OUT = os.path.join(HERE, "output.step")
    try:
        if os.path.exists(OUT):
            os.remove(OUT)
        shape = build()
        if len(shape.Solids) != 1 or not shape.isValid():
            raise RuntimeError("expected one valid solid, got %d" % len(shape.Solids))
        shape = shape.Solids[0]
        shape.exportStep(OUT)
        print("wrote", OUT, "volume %.1f mm3" % shape.Volume, "faces", len(shape.Faces))
    except Exception:
        traceback.print_exc()
        if os.path.exists(OUT):
            os.remove(OUT)
        sys.exit(1)
