"""Connector for facing-L2-s1: a box-section bracket between two facing mounts.

Two 6 mm flanges (one seated on A, one on B) joined by two 6 mm webs that run
between the bolt rows, forming a closed box section.  The whole part is a
prism along one horizontal direction (A's local +Y), so it prints standing on
the flat end face at y = +YB with every wall vertical: no supports needed.
The webs stop short at the far end so the hex key of bolt B1 (which sits
between the webs) can swing out through the open end.
"""
import json
import math
import pathlib

from build123d import (Align, Axis, Box, Circle, Cylinder, Location, Pos,
                       Rectangle, export_step, extrude, fillet)

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


H = OB[2] - OA[2]            # gap between the mounting faces (68.5)
T = 6.0                      # flange thickness = bolt grip
TW = 6.0                     # web thickness
YB = 35.25                   # print-bed end face (1.25 mm inside A's edge)
HALF_A = 35.25               # flange A half width
R_B = 41.0                   # flange B radius (1.25 mm inside B's edge)
HALF_B = 33.5                # flange B half width near the bed end
WEB_X = (-16.0, 10.5)        # web centre lines, between the bolt rows
WEB_Y0 = -18.0               # webs end here: leaves the key of B1 free
D_HOLE = {"M4": 4.5, "M5": 5.5}


def build():
    cx, cy, _ = to_local(OB)
    # flange on A: square, far corners rounded concentric with the bolts
    fa = Pos(0, 0, T / 2) * Box(2 * HALF_A, 2 * YB, T)
    far = [e for e in fa.edges().filter_by(Axis.Z) if e.center().Y < 0]
    fa = fillet(far, HALF_A - 30.0)
    # flange on B: disc, flat on the bed end, straight sides below the
    # widest printable point so the rim never overhangs more than ~35 deg
    sk = Pos(cx, cy) * Circle(R_B)
    sk &= Pos(cx, YB - 100) * Rectangle(2 * HALF_B, 200)
    fb = Pos(0, 0, H - T) * extrude(sk, T)
    part = fa + fb
    for x in WEB_X:
        part += Pos(x, WEB_Y0, T) * Box(
            TW, YB - WEB_Y0, H - 2 * T,
            align=(Align.CENTER, Align.MIN, Align.MIN))
    for b in BOLTS:
        x, y, z = to_local(b["position"])
        size = "M4" if b["id"].startswith("A") else "M5"
        z0 = -1 if z < H / 2 else H - T - 1
        part -= Pos(x, y, z0) * Cylinder(
            D_HOLE[size] / 2, T + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = part.rotate(Axis.Z, YAW).moved(Location(tuple(OA)))
    return part


if __name__ == "__main__":
    p = build()
    s = p.solids()
    assert len(s) == 1 and p.is_valid, (len(s), p.is_valid)
    print("volume", round(p.volume), "bbox", p.bounding_box())
    export_step(s[0], str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
