"""Connector for task pipe-L3-s1: pipe saddle (4x M5) to a flat square mount (4x M4).

Concept (printed standing on the pipe-axis direction, tilted 2.8 deg so that
both the saddle end and the toe of the mounting plate rest on the bed):

* two rails along the pipe, one per bolt row, each with a flat head seat;
* a low saddle web tying the rails together at the bed end;
* one solid bar lying on the bed from the saddle to the mounting plate,
  running up the plate between the two bolt columns as a rib;
* the mounting plate, a 42 deg ramp standing on its toe, with the bar
  rising behind it as a rib to its top edge.

Running this file writes output.step.
"""
import json
import math
import pathlib

import numpy as np
from build123d import (Box, Cylinder, Plane, Polygon, Pos, Vector, export_step,
                       extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
BOLTS = {b["id"]: b for b in POSE["resolved"]["bolts"]}

# ---- design parameters (mm) -------------------------------------------------
TILT = math.radians(2.8)   # bed tilt off the pipe axis
PLATE_T = 3.2              # mounting plate thickness = M4 grip
PLATE_HALF = 17.5          # plate half width (mount is 19, seat must stay < 18)
TOE_U = -17.95             # seat starts here (bed line on the mounting face)
TOE_CUT = 3.0              # blunt the plate toe this far from the bed line
BAR_HW = 8.0               # bar half width (fits between M4 heads at +-12.5)
BAR_H = 16.5               # bar height above the bed
KINK_Y = 15.0              # bar runs straight this far, then turns to the saddle
RAIL_GRIP = 4.0            # rail thickness under the M5 head
RAIL_HW = 5.0              # rail half width
RAIL_END = 4.75            # rail runs this far past the upper bolts
WEB_T = 3.2                # saddle web thickness
WEB_HALF_ANGLE = 29.0      # deg, web reaches into both rails
RIB_END = 3.0              # rib height left at the top edge of the plate
RIB_FOOT = 5.0             # the rib starts rising from the bar here (bed y)
D_A = 5.5                  # M5 clearance hole
D_B = 4.5                  # M4 clearance hole

A = POSE["placements"]["A"]
B = POSE["placements"]["B"]
RA = np.array(A["rotation"]); P0 = np.array(A["origin"])
RB = np.array(B["rotation"]); BC = np.array(B["origin"])
AX, LY, LZ = RA[:, 0], RA[:, 1], RA[:, 2]          # pipe axis, local y, local z
U, V, N = RB[:, 0], RB[:, 1], RB[:, 2]              # plate in-plane u, v, normal
R_PIPE = TASK["fixed_parts"][0]["interface"]["params"]["od"] / 2


def unit(a):
    return a / np.linalg.norm(a)


def vec(a):
    return Vector(*[float(x) for x in a])


def plane(origin, x_dir, z_dir):
    return Plane(origin=vec(origin), x_dir=vec(x_dir), z_dir=vec(z_dir))


# ---- print frame: bed plane through the plate toe line -----------------------
Q = unit(U - (U @ AX) * AX)                 # away from the pipe, square to its axis
UP = math.cos(TILT) * AX - math.sin(TILT) * Q
UP = unit(UP - (UP @ V) * V)                # build direction, square to the toe line
E0 = BC + TOE_U * U                         # middle of the toe line
YP = np.cross(UP, V)                        # along the bed, towards the pipe
BED = plane(E0, V, UP)
BIG = 400.0


def bed_xy(p):
    r = np.asarray(p) - E0
    return float(r @ V), float(r @ YP)


def build():
    # -- bar on the bed, straight between the bolt columns, then to the saddle
    sx, sy = bed_xy(P0 + R_PIPE * LZ - 15 * AX)
    c = unit(np.array([sx, sy - KINK_Y]))
    ex, ey = np.array([sx, sy]) + 8.0 * c
    pts = [(-BAR_HW, -45), (BAR_HW, -45), (BAR_HW, KINK_Y), (ex + BAR_HW, ey),
           (ex - BAR_HW, ey), (-BAR_HW, KINK_Y)]
    bar = extrude(BED * Polygon(*pts, align=None), 60)
    # side view: flat top, then a rib rising to the top edge of the plate
    top = BC + PLATE_HALF * U + (PLATE_T + RIB_END) * N - E0
    ty, tz = float(top @ YP), float(top @ UP)
    k = (tz - BAR_H) / (ty - RIB_FOOT)
    side = plane(E0, YP, V) * Polygon(
        (-60, 0), (200, 0), (200, BAR_H), (RIB_FOOT, BAR_H),
        (-60, BAR_H + k * (-60 - RIB_FOOT)), align=None)
    bar = bar & extrude(side, 50, both=True)
    # nothing behind the mounting face, nothing past the plate's top edge
    bar = bar & (plane(BC, U, N) * Pos(PLATE_HALF - BIG / 2, 0, BIG / 2)
                 * Box(BIG, BIG, BIG))

    # -- mounting plate, standing on its toe
    pf = plane(BC, U, N)
    plate = extrude(pf * Polygon((-26, -PLATE_HALF), (PLATE_HALF, -PLATE_HALF),
                                 (PLATE_HALF, PLATE_HALF), (-26, PLATE_HALF),
                                 align=None), PLATE_T)
    plate = plate & (BED * Pos(0, TOE_CUT - BIG / 2, 0) * Box(BIG, BIG, BIG))

    # -- saddle: rails along each bolt row, low web between them
    part = bar + plate
    for bid in ("A1", "A3"):
        r = np.array(BOLTS[bid]["axis"])
        lo, hi = -30.0, 17.5 + RAIL_END
        r_in, r_out = R_PIPE - 6, R_PIPE + RAIL_GRIP
        o = P0 + 0.5 * (lo + hi) * AX + 0.5 * (r_in + r_out) * r
        part += plane(o, AX, r) * Box(hi - lo, 2 * RAIL_HW, r_out - r_in)
    a = math.radians(WEB_HALF_ANGLE)
    sec = plane(P0 - 40 * AX, LY, AX) * Polygon(
        (0, 0), (80 * math.sin(a), 80 * math.cos(a)),
        (-80 * math.sin(a), 80 * math.cos(a)), align=None)
    web = extrude(sec, 80) & (plane(P0, LY, AX) * Cylinder(R_PIPE + WEB_T, 100))
    web = web & (BED * Pos(0, 0, BAR_H / 2) * Box(BIG, BIG, BAR_H))
    part += web

    # -- cut: the pipe, everything below the bed, the bolt holes
    part -= plane(P0, LY, AX) * Cylinder(R_PIPE, 300)
    part -= BED * Pos(0, 0, -BIG / 2) * Box(BIG, BIG, BIG)
    for b in BOLTS.values():
        dia = D_A if b["size"] == "M5" else D_B
        p, ax = np.array(b["position"]), np.array(b["axis"])
        x = LY if abs(ax @ LY) < 0.9 else AX
        x = unit(x - (x @ ax) * ax)
        part -= plane(p, x, ax) * Cylinder(dia / 2, 80)
    return part


if __name__ == "__main__":
    part = build()
    solids = part.solids()
    assert len(solids) == 1, len(solids)
    export_step(solids[0], str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(solids[0].volume, 1),
          "valid", solids[0].is_valid)
