"""Connector for task facing-L3-s1, as a FreeCAD script.

    freecadcmd submission.py        # writes output.step

Concept: a side-printed, thin-walled box bracket.

* Two upright webs run along one side of A's bolt square.  Web A stands just
  inside the A1-A4 bolt row, web B lies in the plane of bolt B3.
* An end panel closes the channel at the print-bed end and a thin top deck
  (it lies against B) ties the web tops to it, so the open channel cannot
  rack sideways.
* Every B bolt sits on a pad that is carried without bending a thin wall:
  B1 on the end panel between two ribs, B3 beyond the end of web B (in the
  web's own plane), B2 on web A and a second, parallel wall next to it.
* The A bolts sit on foot tabs right beside the web feet.

The part is printed standing on the end panel (plane y = Y_BED in A's local
frame, build direction +y).  All walls, tabs, pads and the deck are then
vertical; every edge that grows outwards while printing does so at OVERHANG
degrees off the build axis, so no support is needed.  A window in the end
panel lets the hex key of B1 swing.

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
    HERE = os.getcwd()
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


GAP = to_local(PL_B["origin"])[2]                  # gap between the faces
H = GAP - 0.003      # top of the part: a hair below B, so the faces touch without overlapping
CB = to_local(PL_B["origin"])[:2]                  # centre of B's round face
LB = {b["id"]: to_local(b["position"]) for b in BOLTS}

# ---- design parameters (mm) -------------------------------------------------
T_A = 3.2            # foot tabs (grip of the M4 bolts)
T_B = 6.2            # head pads (grip of the M5 bolts)
T_W = 0.9            # webs, ribs, panel, deck: two perimeters
D_A = 4.5            # M4 clearance hole
D_B = 5.5            # M5 clearance hole
OVERHANG = 43.0      # steepest growth off the build axis, degrees
S = math.tan(math.radians(OVERHANG))
Z_B = H - T_B        # underside of the head pads
T_D = 1.2            # top deck
Z_D = H - T_D        # underside of the top deck
R_B = 41.2           # B's round face, 1 mm inside its edge (seat limit)

Y_BED = -35.0                    # print bed plane; the part is built towards +y
Y_PANEL = Y_BED + T_W            # inner face of the end panel
WA_O, WA_I = -25.9, -25.0        # web A (clears the A1, A4 heads)
WA2_O, WA2_I = -32.1, -30.9      # second wall next to web A (clears the B2 head)
WB_I, WB_O = 22.88, 24.08        # web B, in the plane of B3
YA_END = 33.8                    # far end of web A and of the A4 tab
YB_END = 35.3                    # far end of web B's foot and of the A3 tab
YB_TOP = 21.1                    # far end of web B's upper part (clears the B3 head)
ZB_FOOT = 24.5                   # height of web B's foot beyond that
YA_SEAT, YB_SEAT = -27.4, -23.0  # where the web tops reach B's face
YA2_END = 25.5                   # far end of the second wall and of the B2 pad
Z_A2 = 38.0                      # the second wall stays above A1's tool space ...
Y_A2 = -26.0                     # ... up to here, then grows down to its foot
YA2_FOOT = 8.0                 # where its foot starts to rise from the rail
X_TAB = 33.8                     # outer edge of the foot tabs (|x|)
Y_TAB = -26.2                    # end of the two tabs on the bed (A1, A2)
Y_STRIP = -31.0                  # foot strip along the bed, under the panel
Y_RAIL, Y_A3 = -1.2, 18.0       # where rail (A4 side) and A3 tab leave the webs
RIB_L = (-10.64, -9.74)          # ribs either side of B1 (window jambs)
RIB_R = (5.46, 6.36)
Y_RIB = -23.5                    # far end of the ribs and of the B1 pad
B1_X0, B1_X1 = -9.0, -0.6        # B1 pad: its foot on the bed
B2_Y0, B2_XO = 2.5, -41.2        # B2 pad: start on web A, outer edge
B3_Y0, B3_XI, B3_XO, B3_YEND = 12.0, 18.0, 28.95, 30.25
B3_CH = (27.75, 27.5)            # B3 pad: corner cut to stay on B's face
YD_A, YD_B = -27.4, -22.0        # where the deck leaves web A / web B
WIN_Z0, WIN_Z1 = 25.0, 50.0      # key window in the end panel


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


def meet(p, q):
    """Apex where an edge rising from p towards +x meets one rising from q towards -x."""
    (xp, yp), (xq, yq) = p, q
    y = ((xq - xp) / S + yp + yq) / 2
    return (xp + S * (y - yp), y)


def build_local():
    parts = []
    # --- walls along the build direction --------------------------------------
    parts.append(prism_x([(Y_BED, 0), (Y_BED, Z_B), (Y_PANEL, Z_B), (YA_SEAT, H),
                          (YA_END, H), (YA_END, 0)], WA_O, WA_I))
    parts.append(prism_x([(Y_BED, 0), (Y_BED, Z_B), (Y_PANEL, Z_B), (YB_SEAT, H), (YB_TOP, H),
                          (YB_TOP, ZB_FOOT), (YB_END, ZB_FOOT), (YB_END, 0)], WB_I, WB_O))
    # second wall: hangs from the panel above A1's tool space, then its lower
    # edge and a foot rising from the rail close the wall (both at OVERHANG)
    ym = ((Z_A2 - T_A) / S + Y_A2 + YA2_FOOT) / 2
    zm = T_A + S * (ym - YA2_FOOT)
    parts.append(prism_x([(Y_BED, Z_A2), (Y_BED, Z_B), (YA2_END, Z_B), (YA2_END, 1.0),
                          (YA2_FOOT - (T_A - 1.0) / S, 1.0), (ym, zm), (Y_A2, Z_A2)], WA2_O, WA2_I))
    for x0, x1 in (RIB_L, RIB_R):
        parts.append(prism_x([(Y_BED, WIN_Z0), (Y_BED, Z_B), (Y_RIB, Z_B), (Y_RIB, WIN_Z0)], x0, x1))
    # --- end panel on the bed ---------------------------------------------------
    parts.append(prism_y([(WA_O, 0), (WB_O, 0), (WB_O, Z_B), (WA2_O, Z_B), (WA2_O, Z_A2),
                          (WA_O, Z_A2)], Y_BED, Y_PANEL))
    # --- foot: strip and tabs on the bed, rail and tab at the far end -----------
    parts.append(prism_z([(-X_TAB, Y_BED), (WA_I, Y_BED), (WA_I, Y_TAB), (-X_TAB, Y_TAB)], 0, T_A))
    parts.append(prism_z([(WB_I, Y_BED), (X_TAB, Y_BED), (X_TAB, Y_TAB), (WB_I, Y_TAB)], 0, T_A))
    parts.append(prism_z([(WA_O, Y_RAIL), (WA2_O, Y_RAIL + (WA_O - WA2_O) / S), (WA2_O, YA_END),
                          (WA_I, YA_END), (WA_I, Y_RAIL)], 0, T_A))
    parts.append(prism_z([(WB_O, Y_A3), (X_TAB, Y_A3 + (X_TAB - WB_O) / S), (X_TAB, YB_END),
                          (WB_I, YB_END), (WB_I, Y_A3)], 0, T_A))
    # --- head pads --------------------------------------------------------------
    b1_l = (RIB_L[0], Y_BED + (B1_X0 - RIB_L[0]) / S)
    b1_r = (RIB_R[1], Y_BED + (RIB_R[1] - B1_X1) / S)
    pad1 = [(B1_X0, Y_BED), (B1_X1, Y_BED), b1_r, (RIB_R[1], Y_RIB), (RIB_L[0], Y_RIB), b1_l]
    pad2 = [(WA_O, B2_Y0), (B2_XO, B2_Y0 + (WA_O - B2_XO) / S), (B2_XO, YA2_END),
            (WA_I, YA2_END), (WA_I, B2_Y0)]
    pad3 = [(WB_I, B3_Y0), (B3_XI, B3_Y0 + (WB_I - B3_XI) / S), (B3_XI, B3_YEND),
            (B3_CH[0], B3_YEND), (B3_XO, B3_CH[1]), (B3_XO, B3_Y0 + (B3_XO - WB_O) / S),
            (WB_O, B3_Y0)]
    # --- top deck: leaves the B1 pad and both webs, closed from YD on -----------
    deck = [(WA_I, YD_A), meet((WA_I, YD_A), b1_l), b1_l, b1_r, meet(b1_r, (WB_I, YD_B)),
            (WB_I, YD_B), (WB_I, B3_YEND), (WA_I, B3_YEND)]
    # everything that lies against B stays on B's face
    for x, y in pad1 + pad2 + pad3 + deck + [(WA_O, YA_SEAT), (WB_O, YB_SEAT), (WA_O, YA_END)]:
        assert math.hypot(x - CB[0], y - CB[1]) <= R_B, ("off B's face", x, y)
    for poly in (pad1, pad2, pad3):
        parts.append(prism_z(poly, Z_B, H))
    parts.append(prism_z(deck, Z_D, H))

    body = parts[0]
    for s in parts[1:]:
        body = body.fuse(s)

    cutters = [prism_y([(RIB_L[1], WIN_Z0), (RIB_R[0], WIN_Z0), (RIB_R[0], WIN_Z1), (RIB_L[1], WIN_Z1)],
                       Y_BED - 1, Y_PANEL)]
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
    OUT = os.path.join(HERE, "scratch", "trim3.step")
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
