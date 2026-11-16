"""Connector bracket for flange-L3-s1 (build123d).

Concept: a C-shaped bracket printed standing up (world +Z is the build
direction, so every mounting face, wall and bolt bore is vertical or
horizontal). A plate on flange A, a plate on block B, and one stiff side
wall joining them on the side where the two plates come closest, leaving
the space in front of every bolt open for the hex keys.

- A plate: seat is a 70 mm disc (inside the 74.5 mm face, 1 mm set-back),
  stepping back to the plate through a 45-degree cone so nothing overhangs.
  The plate's lower edge rises at 45 degrees from the print bed.
- B plate: flat 43 x 93 mm, sitting 1 mm inside the 46 x 96 face.
- Side wall: thick, so the filament infill makes it stiff for little plastic.
"""
import json
import math
import pathlib

from build123d import (Axis, Box, Cone, Cylinder, Location, Part, Plane,
                       Polyline, Pos, Vector, export_step, extrude, make_face)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

Z = Vector(0, 0, 1)
OA = Vector(*FACES["A"]["origin"])
NA = Vector(*FACES["A"]["normal"]).normalized()
UA = Z.cross(NA).normalized()          # in-plane horizontal on A (towards B)
OB = Vector(*FACES["B"]["origin"])
NB = Vector(*FACES["B"]["normal"]).normalized()
UB = Vector(*TASK["poses"][0]["placements"]["B"]["rotation"][0][0:1] or [0])  # placeholder
rotB = TASK["poses"][0]["placements"]["B"]["rotation"]
UB = Vector(rotB[0][0], rotB[1][0], rotB[2][0]).normalized()   # B local x

# ---- parameters -----------------------------------------------------------
Z_BED = -64.5          # lowest point of the part (B seat bottom, 1.5 mm in)
Z_TOP_B = 28.5         # top of B plate / wall
T_A = 5.0              # A plate thickness (grip)
C_A = 1.5              # depth of the seat cone on A
R_SEAT = 35.0          # A seat disc radius
T_B = 5.0              # B plate thickness (grip)
B_HALF_X = 21.5        # B plate half width (face is 23)
WALL_A_IN, WALL_A_OUT = 36.0, 46.0     # wall span on A (s coordinate)
WALL_B_IN, WALL_B_OUT = -21.0, -31.5   # wall span on B (local x)
HOLE_A, HOLE_B = 5.5, 6.6


def pA(s, n, z=Z_BED):
    """Point in A's frame: s along UA, n along NA, at world height z."""
    p = OA + UA * s + NA * n
    return Vector(p.X, p.Y, z)


def pB(x, t, z=Z_BED):
    p = OB + UB * x + NB * t
    return Vector(p.X, p.Y, z)


def zprism(pts, z0, z1):
    pts = [Vector(p.X, p.Y, z0) for p in pts]
    face = make_face(Polyline(*pts, close=True))
    return extrude(face, amount=z1 - z0, dir=(0, 0, 1))


def build() -> Part:
    zc = OA.Z
    # --- A plate (outline in the A plane: s horizontal, zr = z - zc) --------
    plA = Plane(origin=OA + NA * C_A, x_dir=UA, z_dir=NA)
    d = (R_SEAT + C_A) * math.sqrt(2) + 2.0      # 45-deg lower edge offset
    zb = Z_BED - zc
    s_left = -38.0
    zr_top = 38.0
    outline = [(WALL_A_OUT, zb), (-d - zb, zb), (s_left, -d - s_left),
               (s_left, zr_top), (WALL_A_OUT, zr_top)]
    a_plate = extrude(plA * make_face(Polyline(*outline, close=True)),
                      amount=T_A - C_A)
    cone = Plane(origin=OA, x_dir=UA, z_dir=NA) * Cone(
        R_SEAT, R_SEAT + C_A, C_A,
        align=None) .moved(Location((0, 0, C_A / 2)))
    part = a_plate + cone

    # --- B plate -------------------------------------------------------------
    b_plate = zprism([pB(-B_HALF_X, 0), pB(B_HALF_X, 0),
                      pB(B_HALF_X, T_B), pB(-B_HALF_X, T_B)], Z_BED, Z_TOP_B)
    part = part + b_plate

    # --- side wall -----------------------------------------------------------
    wall = zprism([pA(WALL_A_IN, T_A), pA(WALL_A_OUT, C_A),
                   pB(WALL_B_OUT, 1.0), pB(WALL_B_IN, 1.0),
                   pB(WALL_B_IN, T_B)], Z_BED, Z_TOP_B)
    part = part + wall

    # --- bolt holes ----------------------------------------------------------
    for b in BOLTS:
        ax = Vector(*b["axis"]).normalized()
        pos = Vector(*b["position"])
        dia = HOLE_A if b["size"] == "M5" else HOLE_B
        L = 40.0
        hole = Plane(origin=pos - ax * 5, z_dir=ax) * Cylinder(
            dia / 2, L, align=(None, None, None)).moved(Location((0, 0, L / 2)))
        part = part - hole
    return part.clean()


if __name__ == "__main__":
    p = build()
    print("valid", p.is_valid, "solids", len(p.solids()), "vol", round(p.volume, 1))
    export_step(p, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
