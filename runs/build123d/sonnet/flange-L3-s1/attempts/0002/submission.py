import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TA, TB = 4.0, 4.5      # plate thickness (grip >= 3.75 / 4.5)
WT = 4.0               # beam wall thickness
BOX = 18.0             # inner clear width of the box beam
UA, UB = 37.5, -26.0   # inner wall end positions along each face tangent
RA = 6.0               # A pad / outline offset
HEXR = 30.0
ZBOT = -58.0 - 6.9     # bed level (bottom of B plate)
ZTOP = 5.5 + HEXR * math.sin(math.radians(60)) + RA


def frame(part):
    f = FACES[part]
    o = Vector(*f["origin"]); n = Vector(*f["normal"])
    t = Vector(-n.Y, n.X, 0).normalized()
    return o, n, t


def plane_for(part, off=0.0):
    o, n, t = frame(part)
    return Plane(origin=o + n * off, x_dir=t, z_dir=n)


def extr(part, sk, thick, off=0.0):
    with BuildPart() as p:
        with BuildSketch(plane_for(part, off)):
            sk()
        extrude(amount=thick)
    return p.part


def hexpts(R):
    return [(R * math.cos(math.radians(60 * i)), R * math.sin(math.radians(60 * i))) for i in range(6)]


def sk_ring():
    outer = Polygon(*hexpts(HEXR), align=None)
    outer = offset(outer, amount=RA, kind=Kind.ARC)
    inner = Polygon(*hexpts(HEXR - 4.5 / math.cos(math.radians(30)) * 1.0), align=None)
    # inner hexagon: edge distance = 30*cos30 - 4.5 ... keep ring width 6+4.5 on flats
    inner = offset(Polygon(*hexpts(HEXR), align=None), amount=-4.5)
    subtract(inner)


def sk_keel():
    a = HEXR * math.sin(math.radians(60)) + RA  # 31.98 (bottom flat)
    Polygon((-15, -a), (15, -a), (5, -a - 10), (5, -ZBOT_A()), (-5, -ZBOT_A()), (-5, -a - 10), align=None)


def ZBOT_A():
    return 5.5 - ZBOT


def sk_rectB():
    hw, hh = 21.9, 46.9
    Rectangle(2 * hw, 2 * hh)
    # window with 45 degree roof and floor (no flat ceilings)
    Polygon((-8, -22), (0, -30), (8, -22), (8, 22), (0, 30), (-8, 22), align=None, mode=Mode.SUBTRACT)


def halfspace(part, off):
    o, n, t = frame(part)
    return Box(600, 600, 600, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location(Plane(origin=o + n * off, x_dir=t, z_dir=n)))


def tab(part, u0, u1, thick):
    o, n, t = frame(part)
    zc = (ZTOP + ZBOT) / 2
    with BuildPart() as tb:
        with BuildSketch(plane_for(part, 1.0)):
            with Locations(((u0 + u1) / 2, zc - o.Z)):
                Rectangle(abs(u1 - u0), ZTOP - ZBOT)
        extrude(amount=thick - 1.0)
    return tb.part


def beam(pa, pb, outward_ref):
    d = Vector(pb.X - pa.X, pb.Y - pa.Y, 0).normalized()
    perp = Vector(-d.Y, d.X, 0)
    mid = (pa + pb) / 2
    if perp.dot(outward_ref - mid) > 0:
        perp = perp * -1   # perp points away from the plate centres
    # wall centre-lines: inner wall at 0, outer wall at BOX+WT outward
    def wallsolid(off):
        c = (pa + pb) / 2 + perp * off
        L = (pb - pa).length + 60
        ang = math.degrees(math.atan2(d.Y, d.X))
        with BuildPart() as w:
            with BuildSketch(Plane(origin=(0, 0, ZBOT), z_dir=(0, 0, 1))):
                with Locations((c.X, c.Y)):
                    Rectangle(L, WT, rotation=ang)
            extrude(amount=ZTOP - ZBOT)
        return w.part
    s = wallsolid(0) + wallsolid(BOX + WT)
    return s & halfspace("A", 1.0) & halfspace("B", 1.0)


def build():
    oA, nA, tA = frame("A"); oB, nB, tB = frame("B")
    with BuildPart() as pa_:
        with BuildSketch(plane_for("A")):
            sk_ring_outer = offset(Polygon(*hexpts(HEXR), align=None), amount=RA, kind=Kind.ARC)
            inner = offset(Polygon(*hexpts(HEXR), align=None), amount=-4.5)
            add(sk_ring_outer)
            add(inner, mode=Mode.SUBTRACT)
        extrude(amount=TA)
    A = pa_.part
    with BuildPart() as kl:
        with BuildSketch(plane_for("A", 1.0)):
            sk_keel()
        extrude(amount=TA - 1.0)
    keel = kl.part
    B = extr("B", sk_rectB, TB)
    pA = oA + tA * UA
    pB = oB + tB * UB
    bm = beam(pA, pB, oA)
    part = A + keel + B + bm + tab("A", 31.5, 66.0, TA) + tab("B", -21.0, -58.0, TB)
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
