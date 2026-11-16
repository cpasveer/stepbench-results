"""Connector for task flange-L3-s1 (build123d).

Printed standing, world -Z up (the bed is the top edge of the A frame), no support:
  * A side: hexagonal plate through the six M5 bolts, flat edge on the bed,
    lightened by a window with a pointed, self-supporting roof.
  * B side: two rails through the four M6 bolts (each with a tapered foot down to
    the bed) joined by a cross plate where the beam lands.
  * One stout beam, square to B and landing on the centre of A, carries bending
    and torsion. It runs inside the circle of A bolts and between the B bolts, so
    every bolt keeps its tool and key space. Its bed-side is a 50 deg "keel", and
    a thin fin carries the keel line down to the bed, so every surface is
    self-supporting.
"""
import json
import math
import pathlib

from build123d import (BuildPart, BuildSketch, Circle, Mode, Part, Plane, Polygon,
                       Rectangle, Vector, add, export_step, extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
PLACE = TASK["poses"][0]["placements"]

EZ = Vector(0, 0, 1)
OA = Vector(*FACES["A"]["origin"]); NA = Vector(*FACES["A"]["normal"])
OB = Vector(*FACES["B"]["origin"]); NB = Vector(*FACES["B"]["normal"])
TA_DIR = Vector(-NA.Y, NA.X, 0)                       # A in-plane horizontal (u)
XB = Vector(*[r[0] for r in PLACE["B"]["rotation"]])  # B in-plane horizontal (x)

# ---- parameters -------------------------------------------------------------
T_A = 4.0          # A plate thickness (M5 grip >= 3.75)
T_B = 4.8          # B plate thickness (M6 grip >= 4.5)
D_A = 5.5          # M5 clearance hole
D_B = 6.6          # M6 clearance hole
R_HEX = 35.5       # hexagon circumradius (face radius 37.25, 1 mm setback)
BAND = 9.3         # frame band width
K = math.tan(math.radians(50))            # slope of every unsupported surface
Z_BED = OA.Z + R_HEX * math.sqrt(3) / 2   # bed plane (world z); the part hangs below
RAIL_W = 10.0
RAIL_X = 15.0
Z_RAIL_LO = OB.Z - 45.0
Z_SEAT_HI = OB.Z + 46.5                   # top of the B seat (face top is +48)
FOOT_T = 2.4
BEAM_W = 2 * (RAIL_X - RAIL_W / 2)        # the beam fills the gap between the rails
Z_RIDGE = 27.5                            # keel line of the beam
BEAM_C = 10.0                             # height of the full-width part of the beam
Z_BEAM_LO = Z_RIDGE - K * BEAM_W / 2 - BEAM_C
T_FIN = 1.6


def half_space(origin, normal):
    with BuildPart() as p:
        with BuildSketch(Plane(origin=origin, z_dir=normal)):
            Rectangle(600, 600)
        extrude(amount=300)
    return p.part


def u_on_a_front(x):
    """u coordinate (A frame) where the B-frame line x = const meets A's front plane."""
    p0 = OB + XB * x
    y = (T_A - (p0 - OA).dot(NA)) / NB.dot(NA)
    return (p0 + NB * y - OA).dot(TA_DIR)


def a_plate():
    pl = Plane(origin=OA, x_dir=TA_DIR, z_dir=NA)
    s3 = math.sqrt(3)
    hexa = [(R_HEX * math.cos(math.radians(60 * i)), R_HEX * math.sin(math.radians(60 * i)))
            for i in range(6)]
    ri = R_HEX - BAND / math.cos(math.radians(30))
    apex = -25.5
    ux = (s3 * ri - apex) / (K + s3)      # 50 deg roof meets the inner hexagon
    zx = apex + K * ux
    zb = Z_BEAM_LO - OA.Z                 # window edge flush with the beam's underside
    u_r = max(u_on_a_front(-BEAM_W / 2), u_on_a_front(BEAM_W / 2)) + 2.5
    window = [(0, apex), (ux, zx), (ri / 2, ri * s3 / 2), (u_r, ri * s3 / 2), (u_r, zb),
              (-(zb - apex) / K, zb)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*hexa, align=None)
            Polygon(*window, align=None, mode=Mode.SUBTRACT)
        extrude(amount=T_A)
    return p.part


def rail(xc):
    pl = Plane(origin=Vector(OB.X, OB.Y, 0) + XB * (xc + RAIL_W / 2), x_dir=NB, z_dir=-XB)
    prof = [(0, Z_RAIL_LO), (T_B, Z_RAIL_LO), (T_B, Z_BED), (T_B - FOOT_T, Z_BED), (0, Z_SEAT_HI)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*prof, align=None)
        extrude(amount=RAIL_W)
    return p.part


def b_sketch_plane():
    return Plane(origin=Vector(OB.X, OB.Y, 0), x_dir=XB, z_dir=NB)   # local (x, world z)


def cross_plate():
    h = BEAM_W / 2
    xv = (Z_RIDGE - Z_SEAT_HI + K * h) / (2 * K)
    zv = Z_RIDGE - K * xv
    prof = [(-h, Z_BEAM_LO), (h, Z_BEAM_LO), (h, Z_SEAT_HI), (xv, zv), (0, Z_RIDGE),
            (-xv, zv), (-h, Z_SEAT_HI)]
    with BuildPart() as p:
        with BuildSketch(b_sketch_plane()):
            Polygon(*prof, align=None)
        extrude(amount=T_B)
    return p.part


def beam(half_a):
    h = BEAM_W / 2
    prof = [(-h, Z_BEAM_LO), (h, Z_BEAM_LO), (h, Z_RIDGE - K * h), (0, Z_RIDGE), (-h, Z_RIDGE - K * h)]
    with BuildPart() as p:
        with BuildSketch(b_sketch_plane()):
            Polygon(*prof, align=None)
        extrude(amount=90)
        add(half_a, mode=Mode.INTERSECT)
    return p.part


def fin(half_a):
    with BuildPart() as p:
        with BuildSketch(b_sketch_plane().offset(T_B)):
            Polygon((-T_FIN / 2, Z_RIDGE - 2.0), (T_FIN / 2, Z_RIDGE - 2.0), (T_FIN / 2, Z_BED),
                    (-T_FIN / 2, Z_BED), align=None)
        extrude(amount=90)
        add(half_a, mode=Mode.INTERSECT)
    return p.part


def build() -> Part:
    half_a = half_space(OA + NA * T_A, NA)
    part = a_plate() + rail(RAIL_X) + rail(-RAIL_X) + cross_plate() + beam(half_a) + fin(half_a)
    for b in BOLTS:
        dia = D_A if b["size"] == "M5" else D_B
        ax = Vector(*b["axis"])
        with BuildPart() as c:
            with BuildSketch(Plane(origin=Vector(*b["position"]) - ax * 1, z_dir=ax)):
                Circle(dia / 2)
            extrude(amount=30)
        part -= c.part
    return part


if __name__ == "__main__":
    result = build()
    export_step(result, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "solids", len(result.solids()), "valid", result.is_valid,
          "volume", round(result.volume / 1000, 2), "cm3")
