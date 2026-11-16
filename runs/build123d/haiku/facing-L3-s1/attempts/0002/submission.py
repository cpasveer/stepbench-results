"""Connector between fixed part A (square, 4x M4 below) and fixed part B (round, 3x M5 above).

World mm. A face at z=-94 (faces up), B face at z=-25.5 (faces down).

  * a thin-walled tube (OD 28, wall 1.2) fills the 68.5 mm gap on the axis midway
    between the two mount centres and carries the stiffness;
  * at the bottom (on the print bed) four flat arms run from the tube to the M4
    pads; each M4 head sits on its pad, above the arm, clear of the tube;
  * at the top, three webs run from the tube to the M5 pads: a flange at the B face
    and a web under it whose underside slopes at 45 deg, so it prints without support;
  * the M5 heads hang below the top flanges; the M4 heads sit above the bottom arms;
    each socket key swings outward from its bolt into free space.
"""
import json
import math
import pathlib

from build123d import (
    Align, Axis, Box, Cylinder, Location, Part, Plane, Polygon, BuildPart,
    BuildSketch, export_step, extrude,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACES = {f["part"]: f for f in POSE["mounting_faces"]}

ZA = FACES["A"]["origin"][2]           # -94.0
ZB = FACES["B"]["origin"][2]           # -25.5
A_XY = FACES["A"]["origin"][:2]
B_XY = FACES["B"]["origin"][:2]
A_IDS = ["A1", "A2", "A3", "A4"]
B_IDS = ["B1", "B2", "B3"]

# design numbers (mm)
TUBE_OD, TUBE_WALL = 28.0, 1.2
R_IN = 13.0          # arms and webs start inside the tube wall (wall 12.8..14)
T_A = 4.0            # bottom arm thickness = M4 head bearing grip (3..25)
T_B = 4.0            # top flange thickness = M5 head bearing grip (3.75..25)
W_A = 7.0            # bottom arm width  (= M4 pad diameter 3.5 * 2)
W_B = 8.5            # top flange / web width (= M5 pad diameter 4.25 * 2)
HOLE_A, HOLE_B = 4.5, 5.5

# tube axis: midway between the mount centres (they are nearly coaxial)
PX = 0.5 * (A_XY[0] + B_XY[0])
PY = 0.5 * (A_XY[1] + B_XY[1])


def bolt_polar(bid):
    x, y, _ = BOLTS[bid]["position"]
    dx, dy = x - PX, y - PY
    return math.atan2(dy, dx), math.hypot(dx, dy)


def cyl_z(r, z0, h, x, y):
    return Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).move(Location((x, y, z0)))


def plan_slot(theta, s_end, width, z0, h):
    """Flat member in plan: a bar from the tube out to s_end with a round end of the given width."""
    L = s_end - R_IN
    bar = Box(L, width, h, align=(Align.MIN, Align.CENTER, Align.MIN)).move(Location((R_IN, 0, z0)))
    end = cyl_z(width / 2, z0, h, s_end, 0)
    return (bar + end).rotate(Axis.Z, math.degrees(theta)).move(Location((PX, PY, 0)))


def web_xz(theta, profile, width):
    """Vertical web: profile in (s, z) along the bolt direction, extruded across the width."""
    with BuildPart() as bp:
        with BuildSketch(Plane.XZ):
            Polygon(*profile)
        extrude(amount=width / 2, both=True)
    return bp.part.rotate(Axis.Z, math.degrees(theta)).move(Location((PX, PY, 0)))


def build() -> Part:
    # thin-walled tube, open at both ends
    tube = cyl_z(TUBE_OD / 2, ZA, ZB - ZA, PX, PY)
    bore = cyl_z(TUBE_OD / 2 - TUBE_WALL, ZA - 1, ZB - ZA + 2, PX, PY)
    body = tube - bore

    # bottom arms to the M4 pads (flat, sitting on the bed)
    for bid in A_IDS:
        th, s = bolt_polar(bid)
        body += plan_slot(th, s, W_A, ZA, T_A)

    # top flanges to the M5 pads, and sloped webs underneath them
    for bid in B_IDS:
        th, s = bolt_polar(bid)
        body += plan_slot(th, s, W_B, ZB - T_B, T_B)
        s_web = s - W_B / 2                       # web ends where the head keep-out starts
        under = ZB - T_B
        prof = [(R_IN, under), (s_web, under), (R_IN, under - (s_web - R_IN))]
        body += web_xz(th, prof, W_B)

    # clearance holes on every bolt axis, through the pads
    for bid in A_IDS:
        x, y, _ = BOLTS[bid]["position"]
        body -= cyl_z(HOLE_A / 2, ZA - 1, T_A + 2, x, y)
    for bid in B_IDS:
        x, y, _ = BOLTS[bid]["position"]
        body -= cyl_z(HOLE_B / 2, ZB - T_B - 1, T_B + 2, x, y)
    return body


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(part.volume, 1))
