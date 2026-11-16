"""Bracket joining flange A and block B: two thin seat plates joined by a
stiff open box beam (two walls, floor, cross ribs), printed standing upright."""
import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TA, TB = 4.0, 4.5      # seat plate thickness (grip >= 3.75 / 4.5)
WT = 3.2               # beam wall thickness
FT = 3.2               # floor thickness
BOX = 18.0             # clear width of the box beam
UA, UB = 37.0, -26.0   # inner wall centre-line at each face (tangent coord)
HEXR = 30.0
RA = 6.0               # pad radius around A bolts
RAMP = math.tan(math.radians(30))
ZBOT = -58.0 - 6.9     # bed level = bottom of B seat plate
HB = (21.9, 46.9)      # B seat half sizes
ZTOP = 5.5 + HEXR * math.sin(math.radians(60)) + RA
R0A = 36.1             # ramp start radius on A (outline radius is 37.25)
R0B = (21.9, 46.9)


def frame(part):
    f = FACES[part]
    o = Vector(*f["origin"]); n = Vector(*f["normal"])
    t = Vector(-n.Y, n.X, 0).normalized()
    return o, n, t


def plane_for(part, off=0.0):
    o, n, t = frame(part)
    return Plane(origin=o + n * off, x_dir=t, z_dir=n)


def hexpts(R):
    return [(R * math.cos(math.radians(60 * i)), R * math.sin(math.radians(60 * i))) for i in range(6)]


def extr(part, sk, thick):
    with BuildPart() as p:
        with BuildSketch(plane_for(part)):
            sk()
        extrude(amount=thick)
    return p.part


# ------------------------------------------------------------ plate A
def sk_A():
    outer = offset(Polygon(*hexpts(HEXR), align=None), amount=RA, kind=Kind.ARC)
    add(outer)
    D = 22.0
    Polygon((D, 0), (0, D), (-D, 0), (0, -D), align=None, mode=Mode.SUBTRACT)
    # posts under the lower pads (support, tangent to the pad circles)
    za = ZBOT - 5.5
    for u in (-15, 15):
        with Locations((u, (za - 26) / 2 + 0.0)):
            pass
        with BuildSketch(mode=Mode.PRIVATE) as _:
            pass
    for u in (-15, 15):
        with Locations((u, (za + (-25.98)) / 2)):
            Rectangle(12, abs(-25.98 - za))
    # arm to the beam
    with Locations(((26 + 66) / 2, 0)):
        Rectangle(66 - 26, 18)


def sk_B():
    zc = (ZBOT + ZTOP) / 2 - (-18.0)
    Rectangle(2 * HB[0], 2 * HB[1])
    Polygon((-8, -22), (0, -30), (8, -22), (8, 22), (0, 30), (-8, 22), align=None, mode=Mode.SUBTRACT)
    wlo, whi = ZBOT + 18.0, ZTOP + 18.0
    with Locations(((-58.0 - 21.0) / 2, (wlo + whi) / 2)):
        Rectangle(37.0, whi - wlo)


def frustum(part, sk, scale_fn=None):
    """solid occupying n in [0,1] whose section grows 30deg from the seat outline"""
    with BuildPart() as p:
        with BuildSketch(plane_for(part, 0.0)):
            sk(0.0)
        with BuildSketch(plane_for(part, 1.0)):
            sk(RAMP)
        loft()
    return p.part


def cutter(part, sk):
    o, n, t = frame(part)
    pl = plane_for(part)
    below = Box(500, 500, 80, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Location(pl))
    slab = Box(500, 500, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location(pl))
    return below + (slab - frustum(part, sk))


def skfA(g):
    Circle(R0A + g)


def skfB(g):
    Rectangle(2 * (R0B[0] + g), 2 * (R0B[1] + g))


# ------------------------------------------------------------ beam
def beam(pa, pb, ref):
    d = Vector(pb.X - pa.X, pb.Y - pa.Y, 0).normalized()
    perp = Vector(-d.Y, d.X, 0)
    mid = (pa + pb) / 2
    if perp.dot(ref - mid) > 0:
        perp = perp * -1
    L = (pb - pa).length + 60
    ang = math.degrees(math.atan2(d.Y, d.X))
    H = ZTOP - ZBOT

    def slab(along, across, length, width, z0, h):
        c = mid + d * along + perp * across
        with BuildPart() as w:
            with BuildSketch(Plane(origin=(0, 0, z0), z_dir=(0, 0, 1))):
                with Locations((c.X, c.Y)):
                    Rectangle(length, width, rotation=ang)
            extrude(amount=h)
        return w.part

    off_out = BOX + WT
    s = slab(0, 0, L, WT, ZBOT, H) + slab(0, off_out, L, WT, ZBOT, H)
    s = s + slab(0, off_out / 2, L, off_out + WT, ZBOT, FT)
    span = (pb - pa).length
    for k in range(1, 5):
        s = s + slab(-span / 2 + span * k / 5, off_out / 2, WT, off_out, ZBOT, H)
    return s


def build():
    oA, nA, tA = frame("A"); oB, nB, tB = frame("B")
    A = extr("A", sk_A, TA)
    B = extr("B", sk_B, TB)
    pa = oA + tA * UA
    pb = oB + tB * UB
    part = A + B + beam(pa, pb, oA)
    part = part - cutter("A", skfA) - cutter("B", skfB)
    for b in BOLTS:
        dia = 5.5 if b["size"] == "M5" else 6.7
        pos = Vector(*b["position"]); ax = Vector(*b["axis"])
        cyl = Cylinder(dia / 2, 14, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location(Plane(origin=pos - ax * 1.0, z_dir=ax)))
        part = part - cyl
    return part


if __name__ == "__main__":
    p = build()
    print(len(p.solids()), p.volume, p.is_valid)
    export_step(p, str(HERE / "output.step"))
