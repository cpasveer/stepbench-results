"""Facing connector: square flange on A, round flange on B, joined by a
lofted hollow column. Bolt heads sit on the plate faces with open space
around them for the hex key."""
import json
import math
import pathlib

from build123d import (Axis, Circle, Cylinder, Location, Part, Plane, Pos,
                       Rectangle, RectangleRounded, Rot, Sketch, extrude,
                       fillet, loft, export_step)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
PL = POSE["placements"]
BOLTS = POSE["resolved"]["bolts"]

T_A = 6.0      # plate thickness on A (grip)
T_B = 6.0      # plate thickness on B (grip)
SIDE_A = 70.0  # A face is 73 -> 1.5 mm setback
DIA_B = 81.5   # B face is 84.5 -> 1.5 mm setback
COL_OD = 40.0
COL_ID = 30.0
HOLE_A = 4.5
HOLE_B = 5.5


def ang(rot):
    return math.degrees(math.atan2(rot[1][0], rot[0][0]))


def build() -> Part:
    oa = PL["A"]["origin"]
    ob = PL["B"]["origin"]
    za, zb = oa[2], ob[2]
    rot_a = ang(PL["A"]["rotation"])

    plate_a = Pos(oa[0], oa[1], za) * Rot(0, 0, rot_a) * extrude(
        RectangleRounded(SIDE_A, SIDE_A, 8), T_A)
    plate_b = Pos(ob[0], ob[1], zb - T_B) * extrude(Circle(DIA_B / 2), T_B)

    z0, z1 = za + T_A, zb - T_B
    col = loft([Plane.XY.offset(z0).move(Location((oa[0], oa[1], 0))) * Circle(COL_OD / 2),
                Plane.XY.offset(z1).move(Location((ob[0], ob[1], 0))) * Circle(COL_OD / 2)])
    body = plate_a + plate_b + col
    # fillet the column-to-plate junctions
    edges = body.edges().filter_by(lambda e: e.geom_type.name == "CIRCLE")
    joint = [e for e in edges
             if abs(e.radius - COL_OD / 2) < 1e-3 and (abs(e.center().Z - z0) < 1e-3 or abs(e.center().Z - z1) < 1e-3)]
    body = fillet(joint, 6.0)

    # bore through
    bore = Pos(oa[0], oa[1], za - 1) * Cylinder(COL_ID / 2, T_A + 1, align=None)
    bore += loft([Plane.XY.offset(z0).move(Location((oa[0], oa[1], 0))) * Circle(COL_ID / 2),
                  Plane.XY.offset(z1).move(Location((ob[0], ob[1], 0))) * Circle(COL_ID / 2)])
    bore += Pos(ob[0], ob[1], z1) * Cylinder(COL_ID / 2, T_B + 1, align=None)
    body = body - bore

    for b in BOLTS:
        p = b["position"]
        d = HOLE_A if b["size"] == "M4" else HOLE_B
        if b["axis"][2] > 0:
            body -= Pos(p[0], p[1], za - 1) * Cylinder(d / 2, T_A + 2, align=None)
        else:
            body -= Pos(p[0], p[1], zb - T_B - 1) * Cylinder(d / 2, T_B + 2, align=None)
    return body


if __name__ == "__main__":
    part = build()
    print("valid", part.is_valid, "solids", len(part.solids()), "vol", part.volume)
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
