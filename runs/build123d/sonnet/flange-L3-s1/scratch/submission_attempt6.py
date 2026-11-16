"""Bracket joining flange A and block B.

Light seat plates (diamond pads + struts) carry the bolts.  A diamond-section
thin-wall tube (45 degree walls, so it prints without support) runs between
the two bolt-head tool corridors at mid height and sits directly on both
mounting faces; a single vertical keel wall under it brings it down to the bed.
Everything is self-supporting when printed standing on the keel/floor."""
import json, math, pathlib
from build123d import *

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]

TA, TB = 4.0, 8.0        # seat plate thickness (grip >= 3.75 / 4.5)
WT = 3.2                 # tube / keel wall thickness
PN, PU = 44.0, 6.0       # kink of the beam axis, in A's frame (normal, tangent)
LEADA, LEADB = 12.0, 12.0              # the axis leaves each face along its normal for this length
ZBOT = -58.0 - 6.9       # bed level = bottom of the B seat plate
Z0 = 0.4                 # tube axis height
HT = 20.0                # tube half height = half width (45 deg walls)
CW = 12.0                 # width of the vertical bars of plate B
TOPSTRIP = 1.0           # 1 = bar across the top pads
NBTOP = 1.0              # 1 = also a root block above the tube
NB = 14.0                 # depth of the root blocks of plate B (0 = none)
BLK = 9.0                # half width of the root blocks
BARH = 10.0              # height of the bottom bar of plate B
STRIPH = 8.0             # height of the top bar of plate B
DA, DB = 6.0, 6.9        # pad diamond half-diagonals
RAMP = 1.3               # radial run of the 1 mm lift ramp (steeper than 45 deg)
R0A = 36.1               # on A the seat may reach r = 36.25
R0B = (22.0, 47.0)       # on B the seat may reach 22 x 47


def frame(part):
    f = FACES[part]
    o = Vector(*f["origin"]); n = Vector(*f["normal"])
    t = Vector(-n.Y, n.X, 0).normalized()
    return o, n, t


def plane_for(part, off=0.0):
    o, n, t = frame(part)
    return Plane(origin=o + n * off, x_dir=t, z_dir=n)


def hull(points):
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo = []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    up = []
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def dia_pts(c, d):
    return [(c[0] + d, c[1]), (c[0], c[1] + d), (c[0] - d, c[1]), (c[0], c[1] - d)]


def poly(points):
    Polygon(*points, align=None)


PADS_A = {1: (-15, 25.98), 2: (-30, 0), 3: (-15, -25.98), 4: (15, -25.98), 5: (30, 0), 6: (15, 25.98)}


def sk_A():
    for c in PADS_A.values():
        poly(dia_pts(c, DA))
    for a, b in ((5, 6), (5, 4), (2, 1), (2, 3)):
        poly(hull(dia_pts(PADS_A[a], DA) + dia_pts(PADS_A[b], DA)))
    # feet that carry the lower pads onto the keel wall (underside steeper than 45 deg)
    poly(hull([(-15, -31.98), (-9, -25.98), (-0.8, -47.0), (-0.8, -28.0)]))
    poly(hull([(15, -31.98), (9, -25.98), (0.8, -47.0), (0.8, -28.0)]))


def sk_B():
    pads = [(-15, -40), (15, -40), (-15, 40), (15, 40)]
    for c in pads:
        poly(dia_pts(c, DB))
    for u in (-15, 15):
        with Locations((u, 0)):
            Rectangle(CW, 80)
    poly(hull(dia_pts(pads[0], DB) + dia_pts(pads[1], DB)))
    if TOPSTRIP:
        # struts from the top pads down onto the walls of the tube (45+ degree undersides)
        for sx in (-1, 1):
            a = (sx * 15.0, 40.0); b = (sx * 5.0, 29.5)
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy); nx, ny = -dy / L * 3.0, dx / L * 3.0
            poly([(a[0] + nx, a[1] + ny), (b[0] + nx, b[1] + ny), (b[0] - nx, b[1] - ny), (a[0] - nx, a[1] - ny)])


def sk_blocks():
    # root block under the bottom bar, between the pads (stands on the bed)
    with Locations((0, -46.9 + 13.8 / 2)):
        Rectangle(2 * BLK, 13.8)
    if NBTOP:
        # root block over the tube ridge; its underside follows the tube wall (45 deg)
        m = 36.1
        Polygon((-BLK, 46.9), (BLK, 46.9), (BLK, m - BLK), (0, m), (-BLK, m - BLK), align=None)


def extr(part, sk, thick):
    with BuildPart() as p:
        with BuildSketch(plane_for(part)):
            sk()
        extrude(amount=thick)
    return p.part


def frustum(part, sk):
    with BuildPart() as p:
        with BuildSketch(plane_for(part, 0.0)):
            sk(0.0)
        with BuildSketch(plane_for(part, 1.0)):
            sk(RAMP)
        loft()
    return p.part


def cutter(part, sk):
    pl = plane_for(part)
    below = Box(500, 500, 80, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Location(pl))
    slab = Box(500, 500, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location(pl))
    return below + (slab - frustum(part, sk))


def skfA(g):
    Circle(R0A + g)


def skfB(g):
    Rectangle(2 * (R0B[0] + g), 2 * (R0B[1] + g))


# ---------------------------------------------------------------- beam
def offset_poly(pts, d):
    out = []
    n = len(pts)
    dirs = []
    for i in range(n - 1):
        dx, dy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        L = math.hypot(dx, dy)
        dirs.append((dx / L, dy / L))
    for i in range(n):
        if i == 0:
            nx, ny = -dirs[0][1], dirs[0][0]
            out.append((pts[0][0] + nx * d, pts[0][1] + ny * d))
        elif i == n - 1:
            nx, ny = -dirs[-1][1], dirs[-1][0]
            out.append((pts[-1][0] + nx * d, pts[-1][1] + ny * d))
        else:
            n1 = (-dirs[i - 1][1], dirs[i - 1][0]); n2 = (-dirs[i][1], dirs[i][0])
            bx, by = n1[0] + n2[0], n1[1] + n2[1]
            k = 2 * d / (bx * bx + by * by)
            out.append((pts[i][0] + bx * k, pts[i][1] + by * k))
    return out


def prism(polygon, z0, h):
    with BuildPart() as p:
        with BuildSketch(Plane(origin=(0, 0, z0), z_dir=(0, 0, 1))):
            Polygon(*polygon, align=None)
        extrude(amount=h)
    return p.part


def beam_axis():
    oA, nA, tA = frame("A"); oB, nB, tB = frame("B")
    P1 = oA + nA * PN + tA * PU
    P0 = oA + nA * LEADA
    P2 = oB + nB * LEADB
    S = oA - nA * 25
    E = oB - nB * 25
    return [(v.X, v.Y) for v in (S, P0, P1, P2, E)]


def tube(ax):
    """diamond tube swept along the kinked axis (mitred at the kink)"""
    (sx, sy) = ax[0]
    d1 = Vector(ax[1][0] - ax[0][0], ax[1][1] - ax[0][1], 0).normalized()
    lat = Vector(-d1.Y, d1.X, 0)
    k = WT * math.sqrt(2)
    outer = [(HT, 0), (0, HT), (-HT, 0), (0, -HT)]
    inner = [(HT - k, 0), (0, HT - k), (-(HT - k), 0), (0, -(HT - k))]
    pl = Plane(origin=(sx, sy, Z0), x_dir=lat, z_dir=d1)   # local x = lateral, local y = up
    with BuildLine() as path:
        Polyline(*[(x, y, Z0) for x, y in ax])
    with BuildPart() as t:
        with BuildSketch(pl):
            Polygon(*outer, align=None)
            Polygon(*inner, align=None, mode=Mode.SUBTRACT)
        sweep(path=path.wire(), transition=Transition.RIGHT)
    return t.part


def keel(ax):
    lo = offset_poly(ax, -WT / 2); hi = offset_poly(ax, WT / 2)
    ztop = Z0 - HT + 3.0
    return prism(lo + hi[::-1], ZBOT, ztop - ZBOT)


def build():
    A = extr("A", sk_A, TA)
    B = extr("B", sk_B, TB)
    ax = beam_axis()
    part = A + B + tube(ax) + keel(ax)
    if NB > 0:
        part = part + extr("B", sk_blocks, NB)
    part = part - cutter("A", skfA) - cutter("B", skfB)
    for b in BOLTS:
        dia = 5.5 if b["size"] == "M5" else 6.7
        pos = Vector(*b["position"]); axv = Vector(*b["axis"])
        cyl = Cylinder(dia / 2, 14, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location(Plane(origin=pos - axv * 1.0, z_dir=axv)))
        part = part - cyl
    return part


def tool_conflicts(part, extra=0.3):
    tot = 0.0
    for b in BOLTS:
        rad = (4.25 if b["size"] == "M5" else 5.0) + 0.5 + extra
        head = 5.0 if b["size"] == "M5" else 6.0
        pos = Vector(*b["position"]); axv = Vector(*b["axis"])
        cyl = Cylinder(rad, head + 31.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location(Plane(origin=pos + axv * (TA if b["id"][0] == "A" else TB) , z_dir=axv)))
        try:
            v = (part & cyl).volume
        except Exception:
            v = 0.0
        tot += v
        if v > 0.01:
            print("  conflict", b["id"], round(v, 2))
    return tot


if __name__ == "__main__":
    p = build()
    print(len(p.solids()), p.volume, p.is_valid)
    print("tool conflicts", tool_conflicts(p))
    export_step(p, str(HERE / "output.step"))
