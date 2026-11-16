import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TA, TB = 4.0, 4.5   # plate thicknesses (grip min 3.75 / 4.5)
WALL = 4.0
UA, UB = 37.5, -26.0   # wall end positions along each face's tangent
ZTOP, ZBOT = 38.0, -64.5


def frame(part):
    f = FACES[part]
    o = Vector(*f["origin"]); n = Vector(*f["normal"])
    t = Vector(-n.Y, n.X, 0).normalized()
    return o, n, t


def plane_for(part, off=0.0):
    o, n, t = frame(part)
    return Plane(origin=o + n * off, x_dir=t, z_dir=n)


def strip(p0, p1, w):
    d = (p1[0]-p0[0], p1[1]-p0[1])
    L = math.hypot(*d)
    ang = math.degrees(math.atan2(d[1], d[0]))
    with Locations(((p0[0]+p1[0])/2, (p0[1]+p1[1])/2)):
        Rectangle(L, w, rotation=ang)


def sk_hex():
    R = 30.0
    pts = [(R*math.cos(math.radians(60*i)), R*math.sin(math.radians(60*i))) for i in range(6)]
    for i in range(6):
        strip(pts[i], pts[(i+1) % 6], 8)
    for p in pts:
        with Locations(p):
            Circle(6.2)
    for i in (0, 2, 4):
        strip((0, 0), pts[i], 6)
    Polygon([(-19, -30), (19, -30), (4, -45), (4, -70), (-4, -70), (-4, -45)], align=None)


def sk_rect():
    corners = [(-15, -40), (15, -40), (15, 40), (-15, 40)]
    for i in range(4):
        strip(corners[i], corners[(i+1) % 4], 8)
    strip(corners[0], corners[2], 6)
    strip(corners[1], corners[3], 6)
    for c in corners:
        with Locations(c):
            Circle(6.9)


def extr(part, sk, thick, off=0.0):
    with BuildPart() as p:
        with BuildSketch(plane_for(part, off)):
            sk()
        extrude(amount=thick)
    return p.part


def halfspace(part, off):
    o, n, t = frame(part)
    return Box(400, 400, 400, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
        Location(Plane(origin=o + n * off, x_dir=t, z_dir=n)))


def build():
    oA, nA, tA = frame("A"); oB, nB, tB = frame("B")
    A = extr("A", sk_hex, TA)
    B = extr("B", sk_rect, TB)
    pa = oA + tA * UA; pb = oB + tB * UB
    d = pb - pa; d = Vector(d.X, d.Y, 0); L = d.length
    ang = math.degrees(math.atan2(d.Y, d.X))
    mid = (pa + pb) / 2
    with BuildPart() as w:
        with BuildSketch(Plane(origin=(0, 0, ZBOT), x_dir=(1, 0, 0), z_dir=(0, 0, 1))):
            with Locations((mid.X, mid.Y)):
                Rectangle(L + 2 * 12, WALL, rotation=ang)
        extrude(amount=ZTOP - ZBOT)
    wall = w.part & halfspace("A", 1.0) & halfspace("B", 1.0)
    def tab(part, u0, u1, thick):
        o, n, t = frame(part)
        zc = (ZTOP + ZBOT) / 2
        with BuildPart() as tb:
            with BuildSketch(plane_for(part, 1.0)):
                with Locations(((u0 + u1) / 2, zc - o.Z)):
                    Rectangle(u1 - u0, ZTOP - ZBOT)
            extrude(amount=thick - 1.0)
        return tb.part
    part = A + B + wall + tab("A", 31.5, 40.0, TA) + tab("B", -31.0, -21.0, TB)
    # bolt holes
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
