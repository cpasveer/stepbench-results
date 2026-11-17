"""Connector for task facing-L3-s1, as a FreeCAD script.

    freecadcmd submission.py        # writes output.step

Concept: a side-printed truss bracket.  The part is a prismatic truss section
(two upright webs and one diagonal between a foot flange on A and a head
flange on B), extruded along one side of A's bolt square and then trimmed.
It is printed standing on the face x = X_BED (local frame of A): every web and
both flanges are then vertical walls, so nothing needs support.  Edges that
grow outwards while printing do so at OVERHANG degrees off the build axis.

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
T_A = 3.2            # foot flange (grip of the M4 bolts)
T_B = 3.9            # head flange (grip of the M5 bolts)
T_WEB = 1.2          # webs: three 0.4 mm perimeters
D_A = 4.5            # M4 clearance hole
D_B = 5.5            # M5 clearance hole
OVERHANG = 43.0      # steepest growth off the build axis, degrees
SLOPE = math.tan(math.radians(OVERHANG))

X_BED = 35.3         # print bed plane; the part is built towards -x
X_WEB = -12.0        # far end of the webs
X_AEND = -34.0       # far end of the foot rails (A1, A4)
X_BEND = -41.2       # far end of the head tongue (B2)

Y_RAIL_O = 33.8      # foot rails: outer and inner edge (|y|)
Y_RAIL_I = 26.2
W1_O, W1_I = -24.2, -23.0          # upright web 1 (outer, inner face)
W2_I, W2_O = 19.95, 21.15          # upright web 2 (inner, outer face)
D_FOOT, D_HEAD = -17.0, 14.5       # diagonal: centre line at z = 0 and z = H
Y_S1 = -33.4         # head flange edge by B1
Y_S2 = 30.3          # head flange edge by B3
X_S1_END = -6.5      # head flange end beyond B1
X_S1_START = 14.76   # where web 1 starts to touch B
X_S2_START = 33.0    # where web 2 / diagonal start to touch B
X_B3_END = 19.0      # end of the B3 pad
Y_T_I, Y_T_O = 16.3, 25.0          # head tongue to B2

Z_B = H - T_B
BIG = 60.0


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
    m = (D_HEAD - D_FOOT) / H                       # diagonal: dy/dz
    hw = 0.5 * T_WEB * math.sqrt(1 + m * m)         # half width measured along y

    def dl(z):                                       # diagonal faces at height z
        return D_FOOT + m * z - hw

    def dr(z):
        return D_FOOT + m * z + hw

    # --- the truss section, extruded over the whole length -------------------
    section = [
        # foot flange 1: from rail edge to the far face of the diagonal
        [(-Y_RAIL_O, 0), (dr(0), 0), (dr(T_A), T_A), (-Y_RAIL_O, T_A)],
        # foot flange 2
        [(W2_I, 0), (Y_RAIL_O, 0), (Y_RAIL_O, T_A), (W2_I, T_A)],
        # web 1, web 2 (between the flanges; flange 2 and head flanges cap them)
        [(W1_O, T_A), (W1_I, T_A), (W1_I, Z_B), (W1_O, Z_B)],
        [(W2_I, T_A), (W2_O, T_A), (W2_O, Z_B), (W2_I, Z_B)],
        # diagonal
        [(dl(T_A), T_A), (dr(T_A), T_A), (dr(Z_B), Z_B), (dl(Z_B), Z_B)],
        # head flange 1 (over web 1, towards B1)
        [(Y_S1, Z_B), (W1_I, Z_B), (W1_I, H), (Y_S1, H)],
        # head flange 2: from the far face of the diagonal to the B3 pad edge
        [(dl(Z_B), Z_B), (Y_S2, Z_B), (Y_S2, H), (dl(H), H)],
    ]
    body = None
    for poly in section:
        s = prism_x(poly, X_BEND, X_BED)
        body = s if body is None else body.fuse(s)

    cutters = []
    # webs stop at X_WEB
    cutters.append(prism_y([(X_WEB, T_A), (X_WEB, Z_B), (-BIG, Z_B), (-BIG, T_A)], -BIG, BIG))
    # foot flange: beyond the webs only the two rails remain, and they end at X_AEND
    cutters.append(prism_z([(X_WEB, -Y_RAIL_I), (X_WEB, Y_RAIL_I), (-BIG, Y_RAIL_I), (-BIG, -Y_RAIL_I)], -1, T_A))
    cutters.append(prism_z([(X_AEND, -BIG), (X_AEND, BIG), (-BIG, BIG), (-BIG, -BIG)], -1, T_A))
    # bed-side ends rise to B's face at the overhang angle
    for xs, y0, y1 in ((X_S1_START, -BIG, -10.0), (X_S2_START, 0.0, BIG)):
        cutters.append(prism_y([(xs, H), (BIG, H - SLOPE * (BIG - xs)), (BIG, H + 5), (xs, H + 5)], y0, y1))
    # head flange 1: pad for B1 grows out of web 1, ends at X_S1_END
    cutters.append(prism_z([(X_S1_START, W1_O), (BIG, W1_O), (BIG, -BIG),
                            (X_S1_START - (W1_O + BIG) / SLOPE, -BIG)], Z_B, H + 5))
    cutters.append(prism_z([(X_S1_END, W1_O), (X_S1_END, -BIG), (-BIG, -BIG), (-BIG, W1_O)], Z_B, H + 5))
    # head flange 2: pad for B3 grows out of web 2, ends at X_B3_END;
    # further on the flange stays between diagonal and web 2, then turns into
    # the tongue for B2 (which widens at the overhang angle)
    cutters.append(prism_z([(X_S2_START, W2_O), (BIG, W2_O), (BIG, BIG),
                            (X_S2_START - (BIG - W2_O) / SLOPE, BIG)], Z_B, H + 5))
    cutters.append(prism_z([(X_B3_END, W2_O), (X_B3_END, BIG), (-BIG, BIG), (-BIG, Y_T_O),
                            (X_WEB, Y_T_O), (X_WEB + (Y_T_O - W2_O) / SLOPE, W2_O)], Z_B, H + 5))
    cutters.append(prism_z([(X_WEB, 0.0), (X_WEB, Y_T_I), (-BIG, Y_T_I), (-BIG, 0.0)], Z_B, H + 5))
    # clearance holes
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
