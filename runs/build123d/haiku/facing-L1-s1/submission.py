"""Connecting part for facing-L1-s1.

Bridges the square A mount (below, mounting face at z=-94) and the round B
mount (above, mounting face at z=-25.5).

  * base plate, 69 mm square with 4 mm corner radius, on the A face.
    Four M4 clearance holes on the 60 mm square; the heads sit on its top.
  * central round column, 26 mm dia, joining base and flange, with 5 mm
    fillets at both ends.
  * top flange, 80 mm dia disc under the B face. Three M5 clearance holes on
    the 70 mm PCD; the heads hang below it.

The column stands inboard of every bolt, and the outward directions from each
bolt are left open, so the L-keys can swing 60 degrees freely.
"""
import json
import math
import pathlib

from build123d import Axis, Box, Cylinder, GeomType, Location, Part, Pos, Rot, export_step, fillet

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
RES = POSE["resolved"]
FACES = {f["part"]: f for f in RES["mounting_faces"]}
BOLTS = {b["id"]: b for b in RES["bolts"]}

# Mounting faces
A_O = FACES["A"]["origin"]  # (-126.0, 127.5, -94.0), normal +z
B_O = FACES["B"]["origin"]  # (-124.18, 135.11, -25.5), normal -z
Z_A = A_O[2]
Z_B = B_O[2]
# Rotation of the A mount about z: world = Rz(-angle) * local
A_R = POSE["placements"]["A"]["rotation"]
A_ANG = math.degrees(math.atan2(A_R[0][1], A_R[0][0]))  # 55 deg

# Dimensions (mm)
T_A = 8.0  # base plate thickness, grip for M4
T_B = 8.0  # flange thickness, grip for M5
PLATE = 69.0
PLATE_R = 4.0
FLANGE_R = 40.0
COL_R = 13.0
FILLET_R = 5.0
HOLE_A = 4.5  # M4 clearance (4.3-4.8)
HOLE_B = 5.5  # M5 clearance (5.3-5.8)


def _local_of_a(p):
    """World xy of an A-bolt -> coordinates in the A mount's local frame."""
    dx, dy = p[0] - A_O[0], p[1] - A_O[1]
    c, s = A_R[0][0], A_R[0][1]
    return (c * dx - s * dy, s * dx + c * dy)


def build() -> Part:
    # Base plate, built in the A local frame (z from 0 to T_A), then placed.
    # Rounded square, z 0..T_A
    _box = Box(PLATE, PLATE, T_A)
    plate_solid = fillet(_box.edges().filter_by(Axis.Z), PLATE_R)
    plate_solid = plate_solid.moved(Location((0, 0, T_A / 2)))
    for b in ("A1", "A2", "A3", "A4"):
        lx, ly = _local_of_a(BOLTS[b]["position"])
        hole = Pos(lx, ly, T_A / 2) * Cylinder(HOLE_A / 2, T_A + 2)
        plate_solid = plate_solid - hole
    plate_world = Pos(A_O[0], A_O[1], Z_A) * Rot(0, 0, -A_ANG) * plate_solid

    # Top flange under the B face
    bx, by = B_O[0], B_O[1]
    flange = Pos(bx, by, Z_B - T_B / 2) * Cylinder(FLANGE_R, T_B)
    for b in ("B1", "B2", "B3"):
        px, py = BOLTS[b]["position"][0], BOLTS[b]["position"][1]
        flange = flange - Pos(px, py, Z_B - T_B / 2) * Cylinder(HOLE_B / 2, T_B + 2)

    # Central column, between the plate top and the flange underside
    z0 = Z_A + T_A
    z1 = Z_B - T_B
    mx = (A_O[0] + bx) / 2
    my = (A_O[1] + by) / 2
    column = Pos(mx, my, (z0 + z1) / 2) * Cylinder(COL_R, z1 - z0)

    body = plate_world + column + flange

    # Fillet the two column junctions
    edges = []
    for e in body.edges():
        if e.geom_type != GeomType.CIRCLE:
            continue
        if abs(e.radius - COL_R) > 0.05:
            continue
        if abs(e.center().Z - z0) < 0.05 or abs(e.center().Z - z1) < 0.05:
            edges.append(e)
    if edges:
        body = fillet(edges, FILLET_R)
    return body


if __name__ == "__main__":
    part = build()
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
