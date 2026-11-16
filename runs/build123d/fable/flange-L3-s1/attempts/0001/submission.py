"""Connector for task flange-L3-s1 (build123d).

Concept: printed standing, world -Z up (bed at the top edge of the A frame).
  * A side: hexagonal frame plate through the six M5 bolts (flat edge on the bed,
    60 deg sides, window with a self-supporting pointed roof).
  * B side: two rails through the four M6 bolts, each with a small set-back foot
    that reaches the bed.
  * Three thin vertical webs join them. In plan they form a triangulated truss
    (different directions, never crossing); in elevation each is a chevron whose
    notch keeps every bolt's tool/key corridor free and whose edges are all
    >= 50 deg from the bed, so nothing needs support.
"""
import json
import math
import pathlib

from build123d import (Align, Axis, BuildPart, BuildSketch, Circle, Cylinder, Location,
                       Locations, Mode, Part, Plane, Polygon, Rectangle, Vector, add,
                       export_step, extrude)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
PLACE = TASK["poses"][0]["placements"]

EZ = Vector(0, 0, 1)
OA = Vector(*FACES["A"]["origin"]); NA = Vector(*FACES["A"]["normal"])
OB = Vector(*FACES["B"]["origin"]); NB = Vector(*FACES["B"]["normal"])
TA_DIR = Vector(-NA.Y, NA.X, 0)                      # A in-plane horizontal (u)
XB = Vector(*[r[0] for r in PLACE["B"]["rotation"]])  # B in-plane horizontal (x)

# ---- parameters -------------------------------------------------------------
T_A = 4.0          # A plate thickness (M5 grip >= 3.75)
T_B = 4.8          # B rail thickness (M6 grip >= 4.5)
D_A = 5.5          # M5 clearance hole
D_B = 6.6          # M6 clearance hole
R_HEX = 35.5       # hexagon circumradius (face radius 37.25, 1 mm setback)
BAND = 9.3         # frame band width
T_WEB = 1.6        # web thickness
K = math.tan(math.radians(50))   # slope of every unsupported edge
Z_BED = OA.Z + R_HEX * math.sqrt(3) / 2   # print bed plane (world z), part hangs below
RAIL_W = 10.0
Z_RAIL_LO = OB.Z - 45.0
Z_RAIL_SEAT_HI = OB.Z + 47.0     # top of seat (face top - 1 mm)
FOOT_BACK = 1.2
Z_WEB_B = OB.Z + 40 - 6.5        # web top at the rail: just under the upper M6 head


def a_pt(d, u, z=0.0):
    p = OA + NA * d + TA_DIR * u
    return Vector(p.X, p.Y, z)


def b_pt(x, y, z=0.0):
    p = OB + XB * x + NB * y
    return Vector(p.X, p.Y, z)


def a_plate():
    pl = Plane(origin=OA, x_dir=TA_DIR, z_dir=NA)
    hexa = [(R_HEX * math.cos(math.radians(60 * i)), R_HEX * math.sin(math.radians(60 * i)))
            for i in range(6)]
    ri = R_HEX - BAND / math.cos(math.radians(30))
    apex = -25.5
    # roof line (50 deg) from the apex meets the upper inner hexagon edge
    ux = (math.sqrt(3) * ri - apex) / (K + math.sqrt(3))
    zx = apex + K * ux
    hole = [(0, apex), (ux, zx), (ri / 2, ri * math.sqrt(3) / 2),
            (-ri / 2, ri * math.sqrt(3) / 2), (-ux, zx)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*hexa, align=None)
            Polygon(*hole, align=None, mode=Mode.SUBTRACT)
        extrude(amount=T_A)
    return p.part


def rail(xc):
    pl = Plane(origin=OB + XB * (xc + RAIL_W / 2), x_dir=NB, z_dir=-XB)
    z0 = OB.Z
    prof = [(0, Z_RAIL_LO - z0), (T_B, Z_RAIL_LO - z0), (T_B, Z_BED - z0),
            (FOOT_BACK, Z_BED - z0), (FOOT_BACK, Z_RAIL_SEAT_HI + FOOT_BACK * K - z0),
            (0, Z_RAIL_SEAT_HI - z0)]
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*prof, align=None)
        extrude(amount=RAIL_W)
    return p.part


def half_space(origin, normal):
    pl = Plane(origin=origin, z_dir=normal)
    with BuildPart() as p:
        with BuildSketch(pl):
            Rectangle(600, 600)
        extrude(amount=300)
    return p.part


HALF_A = None
HALF_B = None


def web(u_a, x_b, top_a, s1, bottom):
    """Vertical web from the A plate (at u_a) to the rail surface (at x_b).

    top_a/s1: the upper edge is at top_a until s1, then falls at 50 deg; from the
    rail it falls at 50 deg from Z_WEB_B. bottom: [(fraction of length, z)].
    """
    p0 = a_pt(T_A, u_a)
    p1 = b_pt(x_b, T_B)
    L = (p1 - p0).length
    w = (p1 - p0).normalized()
    e = 4.0
    sa = (top_a - Z_WEB_B + K * (L + s1)) / (2 * K)
    za = top_a - K * (sa - s1)
    top = []
    if top_a + K * (e + s1) > Z_BED:           # starts on the bed
        top += [(-e, Z_BED)]
        sb = s1 - (Z_BED - top_a) / K
        top += [(sb, Z_BED)] if sb > -e else []
    else:
        top += [(-e, top_a + K * (e + s1))]
    top += [(sa, za), (L + e, Z_WEB_B + K * e)]
    bot = [(L + e, bottom[-1][1])] + [(f * L, z) for f, z in reversed(bottom)] + [(-e, bottom[0][1])]
    pl = Plane(origin=p0, x_dir=w, z_dir=w.cross(EZ))
    with BuildPart() as p:
        with BuildSketch(pl):
            Polygon(*(top + bot), align=None)
        extrude(amount=T_WEB / 2, both=True)
        add(HALF_A, mode=Mode.INTERSECT)
        add(HALF_B, mode=Mode.INTERSECT)
    return p.part


def build() -> Part:
    global HALF_A, HALF_B
    HALF_A = half_space(OA + NA * T_A, NA)
    HALF_B = half_space(OB + NB * T_B, NB)
    z_lo_a = OA.Z - R_HEX * math.sqrt(3) / 2
    part = a_plate() + rail(15.0) + rail(-15.0)
    # W1: straight on from the frame centre to the near rail
    part += web(-9.0, 15.0, Z_BED, 3.0, [(0, z_lo_a + 3), (0.06, z_lo_a + 3), (0.65, -34.0), (1, -34.0)])
    # WD: the diagonal - lateral stiffness
    part += web(-2.0, -12.0, Z_BED, 14.0,
                [(0, z_lo_a + 3), (0.06, z_lo_a + 3), (0.34, -15.0), (0.66, -15.0), (0.8, -26.0), (0.92, -30.0), (1, -30.0)])
    # WP: outer web - stops the B side turning about the vertical
    part += web(20.9, -18.0, 18.5, 0.0, [(0, -16.0), (0.06, -16.0), (0.52, -39.0), (1, -39.0)])
    for b in BOLTS:
        dia = D_A if b["size"] == "M5" else D_B
        ax = Vector(*b["axis"])
        pl = Plane(origin=Vector(*b["position"]) - ax * 1, z_dir=ax)
        with BuildPart() as c:
            with BuildSketch(pl):
                Circle(dia / 2)
            extrude(amount=30)
        part -= c.part
    return part


if __name__ == "__main__":
    result = build()
    export_step(result, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", "solids", len(result.solids()), "valid", result.is_valid,
          "volume", round(result.volume / 1000, 2), "cm3")
