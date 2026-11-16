"""Connector: central column; flat foot-arms to A on the bottom, sloped
(self-supporting) arms to B on top with counterbored head seats."""
import json, math, pathlib
from build123d import (Part, Plane, Polygon, Cylinder, Box, Pos, Rot, Align, extrude, export_step)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
B = TASK["poses"][0]["resolved"]["bolts"]
PX, PY = -126.0, 127.5           # column axis = centre of A
ZA, ZB = -94.0, -25.5
T = 6.0                           # plate thickness under heads
POCKET = 2.0                      # counterbore depth on B side
HW = 5.0
HWU = 5.55
COLR = 10.0
TAN = math.tan(math.radians(58))
MIN = Align.MIN
CEN = Align.CENTER


def arm(bolt, lower):
    bx, by = bolt["position"][0] - PX, bolt["position"][1] - PY
    D = math.hypot(bx, by)
    ang = math.degrees(math.atan2(by, bx))
    hw = HW if lower else HWU
    e = D + hw
    if lower:
        z0, z1 = ZA, ZA + T
        pts = [(0, z0), (e, z0), (e, z1), (0, z1)]
    else:
        z0, z1 = ZB, ZB - T - POCKET
        x0 = D - 8
        pts = [(0, z0), (e, z0), (e, z1), (x0, z1), (0, z1 - x0 * TAN)]
    prof = Plane.XZ * Polygon(*pts, align=None)
    body = extrude(prof, hw, both=True)
    h = ZB - ZA + 2
    plan = Pos(D, 0, ZA - 1) * Cylinder(hw, h, align=(CEN, CEN, MIN))
    plan = plan + Pos(D / 2, 0, ZA - 1) * Box(D, 2 * hw, h, align=(CEN, CEN, MIN))
    return Rot(0, 0, ang) * (body & plan)


def seg(a, c):
    L = math.hypot(c[0] - a[0], c[1] - a[1])
    ang = math.degrees(math.atan2(c[1] - a[1], c[0] - a[0]))
    sl = Cylinder(HW, T, align=(CEN, CEN, MIN)) + Pos(L / 2, 0, 0) * Box(L, 2 * HW, T, align=(CEN, CEN, MIN)) \
        + Pos(L, 0, 0) * Cylinder(HW, T, align=(CEN, CEN, MIN))
    return Pos(a[0], a[1], ZA) * Rot(0, 0, ang) * sl


def build() -> Part:
    p = Cylinder(COLR, ZB - ZA, align=(CEN, CEN, MIN))
    p = Pos(0, 0, ZA) * p
    for b in B:
        if b["id"][0] == "B":
            p = p + arm(b, False)
    p = Pos(PX, PY, 0) * p
    pos = {b["id"]: (b["position"][0], b["position"][1]) for b in B}
    for a, c in [((PX, PY), pos["A1"]), ((PX, PY), pos["A2"]), ((PX, PY), pos["A4"]), (pos["A2"], pos["A3"])]:
        p = p + seg(a, c)
    bc = TASK["poses"][0]["placements"]["B"]["origin"]
    clip = Pos(bc[0], bc[1], -50) * (Cylinder(90, ZB + 1 + 50, align=(CEN, CEN, MIN)) - Cylinder(41.0, ZB + 1 + 50, align=(CEN, CEN, MIN)))
    p = p - clip
    for b in B:
        x, y, z = b["position"]
        hx, hy, d = x, y, b["clearance_diameter_range"][0]
        if b["id"] == "A3":      # nudge hole 0.08 away from B3 to keep a wall to the B3 arm
            ux, uy = x - pos["B3"][0], y - pos["B3"][1]
            n = math.hypot(ux, uy)
            hx, hy = x + 0.08 * ux / n, y + 0.08 * uy / n
        p = p - Pos(hx, hy, ZA - 1) * Cylinder(d / 2, ZB - ZA + 2, align=(CEN, CEN, MIN))
        if b["id"].startswith("B"):
            p = p - Pos(x, y, -80) * Cylinder(4.75, ZB - T + 80, align=(CEN, CEN, MIN))
            p = p - Pos(x, y, ZA - 1) * Cylinder(3.3, 10, align=(CEN, CEN, MIN))
    return Part(p.clean())


if __name__ == "__main__":
    s = build()
    print(s.volume, s.is_valid, len(s.solids()))
    export_step(s, str(HERE / "output.step"))
