"""Connector for facing-L2-s1: a flanged pedestal ("spool") between two facing mounts.

Square foot on A, hollow round column on B's axis, and a head that flares out
at 40 deg from the vertical to a round flange on B.  It prints foot-down with
no support: the flare is self-supporting, and the B bolt heads sit in round
counterbores cut up into the flare, whose 9.6 mm ceilings bridge cleanly.
The column stays inside both bolt patterns so every hex key swings outward.
"""
import json
import math
import pathlib

from build123d import (Align, Axis, Box, Cylinder, GeomType, Location, Plane,
                       Polygon, Pos, export_step, extrude, fillet, revolve)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
PL = POSE["placements"]
BOLTS = POSE["resolved"]["bolts"]

OA = PL["A"]["origin"]
RA = PL["A"]["rotation"]
OB = PL["B"]["origin"]
YAW = math.degrees(math.atan2(RA[1][0], RA[0][0]))  # A-local frame -> world


def to_local(p):
    dx, dy, dz = p[0] - OA[0], p[1] - OA[1], p[2] - OA[2]
    return (RA[0][0] * dx + RA[1][0] * dy, RA[0][1] * dx + RA[1][1] * dy, dz)


H = OB[2] - OA[2]        # gap between the mounting faces (68.5)
T = 6.0                  # material under every bolt head (grip)
HALF_A = 35.25           # foot half width: 1.25 mm inside A's outline
R_CORNER = 5.25          # foot corners, concentric with the M4 holes
R_B = 41.22              # head flange radius: 1.05 mm inside B's outline
RIM = 8.5                # height of the cylindrical rim of the head
FLARE = 40.0             # flare angle from the vertical (printable overhang)
R_COL = 24.0             # column outer radius
R_BORE = 17.0            # through bore: saves plastic, no sealed hollow
R_FIL = 4.0
D_HOLE = {"M4": 4.5, "M5": 5.5}
Z_CB = 44.0              # counterbores stop here, clear below the flare
D_CBORE = 9.56           # M5 head 8.5 + 0.53 radial clearance; bridges < 10
SPLAY = 9.0              # channel wall angle to the bolt's radial line
ARCH_LEAN = 44.0         # arch roof planes, angle from the vertical
ARCH_YAW = 90.0          # plan direction of the roof planes' normals
FLAT = 0.25              # depth of the rim flat at each channel mouth
R_BOLT = 35.0            # B bolt circle radius
ARCH_DROP = 0.5          # arch apex below the head seat


def cbore(x, y, ang):
    """Counterbore for an M5 head, cut up into the flare from below.

    A plain cylinder would leave a feather-edged lip between itself and the
    flare on the outer side.  Instead the outer side is opened by a channel
    through the rim.  Its side walls are tangent to the counterbore and
    splay slightly more than radial lines, so they leave rim and flare
    square; its roof is a pointed arch of two 44 deg planes, so it needs
    neither bridging nor support and its ridges on the flare climb steeply.
    A shallow flat on the rim lets the arch break out square as well.
    """
    r = D_CBORE / 2
    top = H - T
    tool = Pos(0, 0, top) * Cylinder(
        r, top - Z_CB, align=(Align.CENTER, Align.CENTER, Align.MAX))
    d = math.radians(SPLAY)
    tx, ty = -r * math.sin(d), r * math.cos(d)
    fx, fy = tx + 25 * math.cos(d), ty + 25 * math.sin(d)
    chan = Pos(0, 0, Z_CB) * extrude(
        Polygon((tx, -ty), (fx, -fy), (fx, fy), (tx, ty), align=None),
        top - Z_CB)
    z0 = top - ARCH_DROP
    for sgn in (1, -1):
        a = math.radians(sgn * ARCH_YAW)
        n = (math.cos(a), math.sin(a), math.tan(math.radians(ARCH_LEAN)))
        half = Plane(origin=(r, 0, z0), z_dir=n) * Box(
            200, 200, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
        chan &= half
    # small flat on the rim where the arch breaks out, square to the roof
    flat = Pos(R_B - FLAT - R_BOLT, 0, H - RIM - 2) * Box(
        5, 40, RIM + 4, align=(Align.MIN, Align.CENTER, Align.MIN))
    return Pos(x, y) * (tool + chan + flat).rotate(Axis.Z, ang)


def build():
    cx, cy, _ = to_local(OB)
    foot = Pos(0, 0, T / 2) * Box(2 * HALF_A, 2 * HALF_A, T)
    foot = fillet(foot.edges().filter_by(Axis.Z), R_CORNER)

    z_rim = H - RIM
    z_col = z_rim - (R_B - R_COL) / math.tan(math.radians(FLARE))
    prof = Plane.XZ * Polygon(
        (0, 1), (R_COL, 1), (R_COL, z_col), (R_B, z_rim), (R_B, H), (0, H),
        align=None)
    knee = prof.vertices().sort_by(
        lambda v: abs(v.X - R_COL) + abs(v.Z - z_col))[0]
    prof = fillet(knee, R_FIL)       # blend column into flare
    body = Pos(cx, cy) * revolve(prof, Axis.Z)
    part = foot + body

    # blend the column into the foot
    circ = [e for e in part.edges().filter_by(GeomType.CIRCLE)
            if abs(e.radius - R_COL) < 1e-6 and abs(e.center().Z - T) < 1e-6]
    assert len(circ) == 1, len(circ)
    part = fillet(circ, R_FIL)

    part -= Pos(cx, cy, -1) * Cylinder(
        R_BORE, H + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for b in BOLTS:
        x, y, z = to_local(b["position"])
        if b["id"].startswith("A"):
            part -= Pos(x, y, -1) * Cylinder(
                D_HOLE["M4"] / 2, T + 2,
                align=(Align.CENTER, Align.CENTER, Align.MIN))
        else:
            part -= Pos(x, y, H - T - 1) * Cylinder(
                D_HOLE["M5"] / 2, T + 2,
                align=(Align.CENTER, Align.CENTER, Align.MIN))
            part -= cbore(x, y, math.degrees(math.atan2(y - cy, x - cx)))
    part = part.rotate(Axis.Z, YAW).moved(Location(tuple(OA)))
    return part


if __name__ == "__main__":
    p = build()
    s = p.solids()
    assert len(s) == 1 and p.is_valid, (len(s), p.is_valid)
    print("volume", round(p.volume), "bbox", p.bounding_box())
    export_step(s[0], str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
