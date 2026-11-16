"""Connector for facing-L2-s1: square M4 mount A (below) to round M5 mount B (above).

Concept: a printed pedestal, built upright on the A seat face (which is the
bed face, so the A seat is as flat as the bed).

* A flange: 70 x 70 x 6 square plate (1.5 mm inside the 73 mm mount face),
  r6 corners, four 4.5 mm clearance holes for the M4 screws.
* Column: 28 mm tube on the axis of mount B, rising from the A flange.
* Crown: the column flares into the B flange as a 50-degree cone (40 deg
  overhang, printable without support).  The B seat is a ring face (r 40,
  inside the 84.5 mm mount face) around a lightening well.  Each M5 screw
  sits in a 9.8 mm wide slot cut up into the crown from outside: its roof
  (the screw seat, 6 mm below the mount) is a short bridge between the slot
  walls, and the slot leaves the hex key's long leg a clear swing outward.
* Fillets between column, flange and crown.
"""
import json
import math
import pathlib

from build123d import (
    Align, Axis, Box, Cylinder, Polyline, Pos, RectangleRounded, Rot, Vector,
    export_step, extrude, make_face, revolve,
)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
FACES = POSE["resolved"]["mounting_faces"]
BOLTS = POSE["resolved"]["bolts"]

ZA = FACES[0]["origin"][2]          # A seat (top of mount A), -94
ZB = FACES[1]["origin"][2]          # B seat (bottom of mount B), -25.5
A0 = Vector(*FACES[0]["origin"])
B0 = Vector(*FACES[1]["origin"])
RA = POSE["placements"]["A"]["rotation"]
ANG_A = math.degrees(math.atan2(RA[1][0], RA[0][0]))

T_A = 6.0           # A flange thickness (grip of the M4 screws)
GRIP_B = 6.0        # B seat thickness above the M5 screw heads
A_SIZE = 70.0       # A flange (mount face 73)
R_SEAT = 40.0       # B seat face radius (mount face 42.25, limit 41.25)
R_RIM = 43.0        # crown rim radius below the seat (free space)
Z_RIM = -35.0       # bottom of the rim / top of the cone
SLOPE = math.radians(50)   # cone inclination from horizontal
R_COL = 14.0        # column outer radius
R_BORE = 10.0       # column bore radius
R_WELL = 25.0       # lightening well in the B seat
Z_WELL = -50.0      # depth of the well's cylindrical part
R_POCKET = 4.9      # M5 head pocket (head 8.5 + clearance, bridge < 10)


def z_cone(r):
    return Z_RIM - (R_RIM - r) * math.tan(SLOPE)


def revolved(pts):
    prof = make_face(Polyline(*[(x, 0, z) for x, z in pts], close=True))
    return revolve(prof, Axis.Z, 360)


def crown_and_column():
    """Column + cone + rim, revolved about the B axis (local Z)."""
    ch = R_RIM - R_SEAT
    return revolved([
        (0, ZA + T_A),
        (R_COL, ZA + T_A),
        (R_COL, z_cone(R_COL)),
        (R_RIM, Z_RIM),
        (R_RIM, ZB - ch),
        (R_SEAT, ZB),
        (0, ZB),
    ])


def well():
    zb = Z_WELL - (R_WELL - R_BORE) * math.tan(SLOPE)
    return revolved([(0, ZB + 1), (R_WELL, ZB + 1), (R_WELL, Z_WELL),
                     (R_BORE, zb), (0, zb)])


def circle_edge(part, r, z):
    for e in part.edges():
        bb = e.bounding_box()
        if abs(bb.min.Z - z) > 1e-3 or abs(bb.max.Z - z) > 1e-3:
            continue
        try:
            if abs(e.radius - r) < 1e-3:
                return e
        except Exception:
            pass
    raise LookupError((r, z))


def build():
    off = A0 - B0
    plate = Pos(off.X, off.Y, ZA) * Rot(0, 0, ANG_A) * extrude(
        RectangleRounded(A_SIZE, A_SIZE, 6.0), T_A)
    part = (plate + crown_and_column()).clean()

    part = part.fillet(5.0, [circle_edge(part, R_COL, ZA + T_A)])
    part = part.fillet(8.0, [circle_edge(part, R_COL, z_cone(R_COL))])
    part = part.fillet(4.0, [circle_edge(part, R_RIM, Z_RIM)])

    # bore and lightening well
    part -= Pos(0, 0, ZA - 1) * Cylinder(
        R_BORE, ZB - ZA + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= well()

    # screw holes and M5 head pockets
    for b in BOLTS:
        p = Vector(*b["position"]) - B0
        if b["size"] == "M4":
            part -= Pos(p.X, p.Y, ZA - 1) * Cylinder(
                2.25, T_A + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
        else:
            part -= Pos(p.X, p.Y, ZB - GRIP_B - 1) * Cylinder(
                2.75, GRIP_B + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
            # head pocket, opened radially through the rim as a slot so its
            # walls meet the cone square (no feather edges) and the key's
            # long leg has a clear exit
            ang = math.degrees(math.atan2(p.Y, p.X))
            pocket = Cylinder(R_POCKET, 40, align=(Align.CENTER, Align.CENTER, Align.MAX))
            pocket += Box(R_RIM + 5, 2 * R_POCKET, 40,
                          align=(Align.MIN, Align.CENTER, Align.MAX))
            part -= Pos(p.X, p.Y, ZB - GRIP_B) * Rot(0, 0, ang) * pocket

    part = part.clean()
    return Pos(B0.X, B0.Y, 0) * part


if __name__ == "__main__":
    p = build()
    print("valid", p.is_valid, "solids", len(p.solids()), "vol", round(p.volume))
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
