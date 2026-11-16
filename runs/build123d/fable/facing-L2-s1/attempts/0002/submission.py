"""Connector for facing-L2-s1: a flanged pedestal ("spool") between two facing mounts.

Square foot on A, hollow round column on B's axis, and a head that flares out
at 40 deg from the vertical to a round flange on B.  It prints foot-down with
no support: the flare is self-supporting, and the B bolt heads sit in round
counterbores cut up into the flare, whose 9.6 mm ceilings bridge cleanly.
The column stays inside both bolt patterns so every hex key swings outward.
"""
import json
import math
import pathlib

from build123d import (Align, Axis, Box, Cylinder, GeomType, Location, Plane,
                       Polygon, Pos, export_step, fillet, revolve)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
PL = POSE["placements"]
BOLTS = POSE["resolved"]["bolts"]

OA = PL["A"]["origin"]
RA = PL["A"]["rotation"]
OB = PL["B"]["origin"]
YAW = math.degrees(math.atan2(RA[1][0], RA[0][0]))  # A-local frame -> world


def to_local(p):
    dx, dy, dz = p[0] - OA[0], p[1] - OA[1], p[2] - OA[2]
    return (RA[0][0] * dx + RA[1][0] * dy, RA[0][1] * dx + RA[1][1] * dy, dz)


H = OB[2] - OA[2]        # gap between the mounting faces (68.5)
T = 6.0                  # material under every bolt head (grip)
HALF_A = 35.25           # foot half width: 1.25 mm inside A's outline
R_CORNER = 5.25          # foot corners, concentric with the M4 holes
R_B = 41.2               # head flange radius: 1.05 mm inside B's outline
RIM = 7.0                # height of the cylindrical rim of the head
FLARE = 40.0             # flare angle from the vertical (printable overhang)
R_COL = 24.0             # column outer radius
R_BORE = 17.0            # through bore: saves plastic, no sealed hollow
R_FIL = 4.0
D_HOLE = {"M4": 4.5, "M5": 5.5}
Z_CB = 44.0              # counterbores stop here, clear below the flare
D_CBORE = 9.6            # M5 head 8.5 + 0.55 radial clearance; bridges < 10


def build():
    cx, cy, _ = to_local(OB)
    foot = Pos(0, 0, T / 2) * Box(2 * HALF_A, 2 * HALF_A, T)
    foot = fillet(foot.edges().filter_by(Axis.Z), R_CORNER)

    z_rim = H - RIM
    z_col = z_rim - (R_B - R_COL) / math.tan(math.radians(FLARE))
    prof = Plane.XZ * Polygon(
        (0, 1), (R_COL, 1), (R_COL, z_col), (R_B, z_rim), (R_B, H), (0, H),
        align=None)
    knee = prof.vertices().sort_by(
        lambda v: abs(v.X - R_COL) + abs(v.Z - z_col))[0]
    prof = fillet(knee, R_FIL)       # blend column into flare
    body = Pos(cx, cy) * revolve(prof, Axis.Z)
    part = foot + body

    # blend the column into the foot
    circ = [e for e in part.edges().filter_by(GeomType.CIRCLE)
            if abs(e.radius - R_COL) < 1e-6 and abs(e.center().Z - T) < 1e-6]
    assert len(circ) == 1, len(circ)
    part = fillet(circ, R_FIL)

    part -= Pos(cx, cy, -1) * Cylinder(
        R_BORE, H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for b in BOLTS:
        x, y, z = to_local(b["position"])
        if b["id"].startswith("A"):
            part -= Pos(x, y, -1) * Cylinder(
                D_HOLE["M4"] / 2, T + 2,
                align=(Align.CENTER, Align.CENTER, Align.MIN))
        else:
            part -= Pos(x, y, H - T - 1) * Cylinder(
                D_HOLE["M5"] / 2, T + 2,
                align=(Align.CENTER, Align.CENTER, Align.MIN))
            part -= Pos(x, y, H - T) * Cylinder(
                D_CBORE / 2, H - T - Z_CB, align=(Align.CENTER, Align.CENTER, Align.MAX))
    part = part.rotate(Axis.Z, YAW).moved(Location(tuple(OA)))
    return part


if __name__ == "__main__":
    p = build()
    s = p.solids()
    assert len(s) == 1 and p.is_valid, (len(s), p.is_valid)
    print("volume", round(p.volume), "bbox", p.bounding_box())
    export_step(s[0], str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
