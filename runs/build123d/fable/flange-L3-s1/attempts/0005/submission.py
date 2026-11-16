"""Connector for task flange-L3-s1 (build123d).

Printed standing, world -Z up (the bed is the top edge of the A plate), no support:
  * A side: hexagonal plate through the six M5 bolts, flat edge on the bed.
  * B side: two rails through the four M6 bolts, each with a tapered foot to the bed.
  * Between them one stout solid beam, square to B, as large as the bolts' tool and
    key corridors allow: it carries bending in both directions and torsion.  Its
    bed-side is a 50 deg "keel" and a thin fin carries the keel line down to the
    bed, so every surface is self-supporting.
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

OA = Vector(*FACES["A"]["origin"]); NA = Vector(*FACES["A"]["normal"])
OB = Vector(*FACES["B"]["origin"]); NB = Vector(*FACES["B"]["normal"])
TA_DIR = Vector(-NA.Y, NA.X, 0)                       # A in-plane horizontal (u)
XB = Vector(*[r[0] for r in PLACE["B"]["rotation"]])  # B in-plane horizontal (x)

# ---- parameters -------------------------------------------------------------
T_A = 4.0          # A plate thickness (M5 grip >= 3.75)
T_B = 4.8          # B rail thickness (M6 grip >= 4.5)
D_A = 5.5          # M5 clearance hole
D_B = 6.6          # M6 clearance hole
R_HEX = 35.0       # hexagon circumradius: flats 4.33 mm outside the bolt circle (head 4.25)
K = math.tan(math.radians(50))            # slope of every unsupported surface
Z_BED = OA.Z + R_HEX * math.sqrt(3) / 2   # bed plane (world z); the part hangs below
RAIL_W = 10.0
RAIL_X = 15.0
Z_RAIL_LO = OB.Z - 45.0
Z_SEAT_HI = OB.Z + 46.5                   # top of the B seat (face top is +48)
FOOT_T = 2.4
# beam, in B's frame (x across the face, world z); it is extruded square to B
X_RIDGE, Z_RIDGE = 1.0, 27.5              # keel line: between the upper bolts of A and of B
X_LO, X_HI = -20.0, 20.0                  # side walls: flush with the outer sides of the rails
Z_BEAM_LO = 0.0                           # underside of the beam
Z_DEEP = Z_BEAM_LO                        # no deeper keel (kept as an option)
U_WALL = 24.8                             # wall along A's axis, just inside bolt A5's corridor
Z_BEND = Z_BEAM_LO                        # underside of the B end
U_KEEL = 9.9                              # lower keel between the corridors of A's lower bolts
D_FREE = 40.3                             # beyond this distance from A, A's tool corridors end
T_FIN = 1.2
U_BUT = 0.0                               # buttress on the bed between A's two upper bolts
L_BUT = 2.0                               # its flat length on the bed before it slopes away
X_GUS = 9.4                               # gusset under the beam, between B's lower bolts
Y_GUS = 15.0                              # its reach along the beam
X_BB = 9.4                                # buttress on the bed between B's two upper bolts
L_BB = 2.0                                # its flat length on the bed before it slopes away
Z_FILL = Z_SEAT_HI - 2.0                  # where the fillets between rails and keel roof start


def half_space(origin, normal):
    with BuildPart() as p:
        with BuildSketch(Plane(origin=origin, z_dir=normal)):
            Rectangle(600, 600)
        extrude(amount=300)
    return p.part


def a_plate():
    pl = Plane(origin=OA, x_dir=TA_DIR, z_dir=NA)
    hexa = [(R_HEX * math.cos(math.radians(60 * i)), R_HEX * math.sin(math.radians(60 * i)))
            for i in range(6)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*hexa, align=None)
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


def prism(z_lo, z_top=None):
    """Beam profile extruded square to B: keel roof on the bed side, flat underside at z_lo."""
    if z_top is None:
        prof = [(X_LO, z_lo), (X_HI, z_lo), (X_HI, Z_RIDGE - K * (X_HI - X_RIDGE)),
                (X_RIDGE, Z_RIDGE), (X_LO, Z_RIDGE - K * (X_RIDGE - X_LO))]
    else:
        prof = [(X_LO, z_lo), (X_HI, z_lo), (X_HI, z_top), (X_LO, z_top)]
    with BuildPart() as p:
        with BuildSketch(b_sketch_plane()):
            Polygon(*prof, align=None)
        extrude(amount=95)
    return p.part


def hex_prism():
    hexa = [(R_HEX * math.cos(math.radians(60 * i)), R_HEX * math.sin(math.radians(60 * i)))
            for i in range(6)]
    with BuildPart() as p:
        with BuildSketch(Plane(origin=OA, x_dir=TA_DIR, z_dir=NA)):
            Polygon(*hexa, align=None)
        extrude(amount=120)
    return p.part


def beam():
    in_front_of_a = half_space(OA, NA)
    # waist: where A's tool corridors run, the section stays inside them
    waist = prism(Z_BEAM_LO) & half_space(OA + TA_DIR * U_WALL, -TA_DIR) & hex_prism()
    # lower keel between the corridors of A's two lower bolts
    keel = (prism(Z_DEEP, Z_BEAM_LO + 1) & half_space(OA + TA_DIR * U_KEEL, -TA_DIR)
            & half_space(OA - TA_DIR * U_KEEL, TA_DIR) & half_space(OA + NA * (D_FREE + 1), -NA))
    # B end: past A's corridors the full section is free
    b_end = prism(Z_BEND) & half_space(OA + NA * D_FREE, NA)
    body = waist + b_end
    if Z_DEEP < Z_BEAM_LO - 0.01:
        body += keel
    return body & in_front_of_a


def fillets():
    """Plate between the rails above the keel roof: turns the two 40 deg valleys between
    roof and rails into 80 deg ones (each face still grows at 50 deg from a wall)."""
    h = RAIL_X - RAIL_W / 2
    xr = (Z_RIDGE - Z_FILL + K * (h + X_RIDGE)) / (2 * K)
    xl = (Z_FILL - Z_RIDGE + K * (X_RIDGE - h)) / (2 * K)
    prof = [(-h, Z_RIDGE - K * (X_RIDGE + h) - 3), (X_RIDGE, Z_RIDGE - 3), (h, Z_RIDGE - K * (h - X_RIDGE) - 3),
            (h, Z_FILL), (xr, Z_RIDGE - K * (xr - X_RIDGE)),
            (X_RIDGE, Z_RIDGE), (xl, Z_RIDGE - K * (X_RIDGE - xl)), (-h, Z_FILL)]
    with BuildPart() as p:
        with BuildSketch(b_sketch_plane()):
            Polygon(*prof, align=None)
        extrude(amount=T_B)
    return p.part


def buttress():
    """Block on the bed between A's upper bolts: ties them to the beam."""
    z_low = Z_RIDGE - K * 16
    pl = Plane(origin=Vector(OA.X, OA.Y, 0), x_dir=NA, z_dir=-TA_DIR)
    prof = [(0, z_low), (0, Z_BED), (T_A + L_BUT, Z_BED), (T_A + L_BUT + (Z_BED - z_low) / K, z_low)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*prof, align=None)
        extrude(amount=U_BUT, both=True)
    return p.part


def gusset():
    """Wedge under the beam between B's lower bolts, on a plate joining the rails."""
    pl = Plane(origin=Vector(OB.X, OB.Y, 0) + XB * X_GUS, x_dir=NB, z_dir=-XB)
    prof = [(0, Z_RAIL_LO), (T_B + 2, Z_RAIL_LO), (Y_GUS, Z_BEND), (Y_GUS, Z_BEND + 1), (0, Z_BEND + 1)]
    h = RAIL_X - RAIL_W / 2
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*prof, align=None)
        extrude(amount=2 * X_GUS)
        with BuildSketch(b_sketch_plane()):
            Polygon((-h, Z_RAIL_LO), (h, Z_RAIL_LO), (h, Z_BEND + 1), (-h, Z_BEND + 1), align=None)
        extrude(amount=T_B)
    return p.part


def b_buttress():
    """Block on the bed between B's upper bolts (ties them to the beam), on a plate that
    joins the two rails; both lean back off B's face above the seat."""
    z_low = Z_RIDGE - K * 12
    y0 = T_B + L_BB
    pl = Plane(origin=Vector(OB.X, OB.Y, 0) + XB * X_BB, x_dir=NB, z_dir=-XB)
    prof = [(0, z_low), (0, Z_SEAT_HI), (T_B - FOOT_T, Z_BED), (y0, Z_BED), (y0 + (Z_BED - z_low) / K, z_low)]
    h = RAIL_X - RAIL_W / 2
    pl2 = Plane(origin=Vector(OB.X, OB.Y, 0) + XB * h, x_dir=NB, z_dir=-XB)
    prof2 = [(0, z_low), (0, Z_SEAT_HI), (T_B - FOOT_T, Z_BED), (T_B, Z_BED), (T_B, z_low)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*prof, align=None)
        extrude(amount=2 * X_BB)
        with BuildSketch(pl2):
            Polygon(*prof2, align=None)
        extrude(amount=2 * h)
    return p.part


def fin(half_a):
    """Thin web under the keel line, down to the bed; its B end leans back off B's face."""
    pl = Plane(origin=Vector(OB.X, OB.Y, 0) + XB * (X_RIDGE + T_FIN / 2), x_dir=NB, z_dir=-XB)
    prof = [(0, Z_RIDGE - 2), (0, Z_RIDGE), (T_B, Z_BED), (90, Z_BED), (90, Z_RIDGE - 2)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*prof, align=None)
        extrude(amount=T_FIN)
        add(half_a, mode=Mode.INTERSECT)
    return p.part


def build() -> Part:
    half_a = half_space(OA + NA * T_A, NA)
    part = a_plate() + rail(RAIL_X) + rail(-RAIL_X) + beam() + fin(half_a)
    part += b_buttress() if X_BB > 0 else fillets()
    if U_BUT > 0:
        part += buttress()
    if Y_GUS > 0:
        part += gusset()
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
