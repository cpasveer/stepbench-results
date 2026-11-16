"""Connector between fixed part A (square, 4x M4 below) and fixed part B (round, 3x M5 above).

Layout (world mm, A face at z=-94 facing up, B face at z=-25.5 facing down):
  * a thin-walled round tube (OD 22, wall 1.2) runs the full 68.5 mm gap, on the
    axis midway between the two mount centres; it carries the stiffness;
  * at the bottom, four flat arms (on the print bed) run from the tube to the M4
    bolt pads; their heads sit on top of the pads, clear of the tube;
  * at the top, three radial wedges run from the tube to the M5 pads; the
    underside of each wedge is a 45 deg slope so the part prints without support;
  * the M5 heads sit below the top wedges and the M4 heads above the bottom arms,
    so each socket key can swing outward into free space.
"""
import json
import math
import pathlib

from build123d import (
    BuildPart, BuildSketch, Cylinder, Plane, Polygon, Pos, Rot, Location,
    Box, Part, export_step, extrude, Mode,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACES = {f["part"]: f for f in POSE["mounting_faces"]}

ZA = FACES["A"]["origin"][2]          # -94.0, A face (outward +Z)
ZB = FACES["B"]["origin"][2]          # -25.5, B face (outward -Z)
A_ORIGIN = FACES["A"]["origin"][:2]
B_ORIGIN = FACES["B"]["origin"][:2]
A_IDS = ["A1", "A2", "A3", "A4"]
B_IDS = ["B1", "B2", "B3"]

# design numbers
TUBE_OD, TUBE_WALL = 22.0, 1.2
T_A = 4.0            # bottom flange thickness = head bearing grip at M4
T_B = 4.0            # top wedge flange thickness = head bearing grip at M5
ARM_W = 6.0          # bottom arm width
WEDGE_W = 4.0        # top wedge width (tangential)
PAD_R = 4.75         # pad radius around each bolt (head radius 3.5 / 4.25 + clearance)
HOLE_A = 4.5         # clearance for M4 (4.3-4.8)
HOLE_B = 5.5         # clearance for M5 (5.3-5.8)
R_TUBE_IN_START = 10.0   # arms and wedges start inside the tube wall

# tube axis: midway between the two mount centres (both are nearly coaxial)
PX = 0.5 * (A_ORIGIN[0] + B_ORIGIN[0])
PY = 0.5 * (A_ORIGIN[1] + B_ORIGIN[1])


def polar(bolt_id):
    x, y, _ = BOLTS[bolt_id]["position"]
    dx, dy = x - PX, y - PY
    s = math.hypot(dx, dy)
    return math.atan2(dy, dx), s


def prism_xz(theta, profile, half_width):
    """Profile given in (s, z) with s along the direction theta from the tube axis."""
    with BuildPart() as bp:
        with BuildSketch(Plane.XZ):
            Polygon(*profile)
        extrude(amount=half_width, both=True)
    return (bp.part.rotate(__import__("build123d").Axis.Z, math.degrees(theta))
            .move(Location((PX, PY, 0))))


def build() -> Part:
    # tube: thin-walled, open at both ends
    tube = (Cylinder(TUBE_OD / 2, ZB - ZA, align=(
        __import__("build123d").Align.CENTER,
        __import__("build123d").Align.CENTER,
        __import__("build123d").Align.MIN)).move(Pos(PX, PY, ZA)))
    bore = (Cylinder(TUBE_OD / 2 - TUBE_WALL, ZB - ZA + 2, align=(
        __import__("build123d").Align.CENTER,
        __import__("build123d").Align.CENTER,
        __import__("build123d").Align.MIN)).move(Pos(PX, PY, ZA - 1)))
    body = tube - bore

    # bottom arms to the M4 pads (flat, printed on the bed)
    for bid in A_IDS:
        th, s = polar(bid)
        prof = [(R_TUBE_IN_START, ZA), (s, ZA), (s, ZA + T_A), (R_TUBE_IN_START, ZA + T_A)]
        body += prism_xz(th, prof, ARM_W / 2)
        x, y, _ = BOLTS[bid]["position"]
        body += Pos(x, y, ZA) * Cylinder(PAD_R, T_A, align=(
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.MIN))

    # top wedges to the M5 pads: flange at the B face, underside at 45 deg toward the tube
    for bid in B_IDS:
        th, s = polar(bid)
        zb_under = ZB - T_B
        drop = (s - PAD_R) - R_TUBE_IN_START
        prof = [
            (R_TUBE_IN_START, ZB),
            (s, ZB),
            (s, zb_under),
            (s - PAD_R, zb_under),
            (R_TUBE_IN_START, zb_under - drop),
        ]
        body += prism_xz(th, prof, WEDGE_W / 2)
        x, y, _ = BOLTS[bid]["position"]
        body += Pos(x, y, ZB - T_B) * Cylinder(PAD_R, T_B, align=(
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.MIN))

    # clearance holes on every bolt axis
    for bid in A_IDS:
        x, y, _ = BOLTS[bid]["position"]
        body -= Pos(x, y, ZA - 1) * Cylinder(HOLE_A / 2, T_A + 2, align=(
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.MIN))
    for bid in B_IDS:
        x, y, _ = BOLTS[bid]["position"]
        body -= Pos(x, y, ZB - T_B - 1) * Cylinder(HOLE_B / 2, T_B + 2, align=(
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.CENTER,
            __import__("build123d").Align.MIN))
    return body


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "volume", round(part.volume, 1))
